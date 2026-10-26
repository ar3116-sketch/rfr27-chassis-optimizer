using System.Text.Json;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    static object ExportStudyAudit(CWStudy s,string folder) {
        var target=Path.Combine(folder,"native-deck");Directory.CreateDirectory(target);
        int code=s.ExportSimulationStudy(target,"audit",(int)swsNastranExportOption_e.swsNastranExportOption_LongFree,0,0,
            (int)swsStudyExportOption_e.swsStudyExportOption_Nastran,(int)swsNastranExportUnit_e.swsNastranExportUnit_SI);
        return new {code,format="Nastran audit export only; solve remains native SolidWorks Simulation",files=Directory.GetFiles(target).Select(p=>new {path=p,sha256=FileSha(p)}).ToArray()};
    }
    static object FrameAcceptance(JsonElement job) {
        string output=Safe(job.GetProperty("output").GetString()!),restore=Safe(job.GetProperty("restore").GetString()!);
        if(!output.Contains(Path.Combine("cad","tests"))||File.Exists(output))throw new Exception("Fresh test path required");
        double scale=job.TryGetProperty("loadScale",out var sc)?sc.GetDouble():1;
        if(!double.IsFinite(scale)||Math.Abs(scale)>2||scale==0)throw new Exception("Acceptance load scale out of range");
        var points=(from x in new[]{0d,1d} from y in new[]{-.25,.25} from z in new[]{-.2,.2} select new[]{x,y,z}).ToArray();
        var pairs=(from i in Enumerable.Range(0,8) from j in Enumerable.Range(i+1,7-i) where Enumerable.Range(0,3).Count(k=>points[i][k]!=points[j][k])==1 select (i,j)).ToArray();
        var d=New(false);
        try {
            d.FeatureManager.InsertWeldmentFeature();d.SketchManager.Insert3DSketch(true);d.SketchManager.AddToDB=true;
            var lines=pairs.Select(p=>{var a=points[p.i];var b=points[p.j];return d.SketchManager.CreateLine(a[0],a[1],a[2],b[0],b[1],b[2]);}).ToArray();
            d.SketchManager.AddToDB=false;d.SketchManager.Insert3DSketch(true);d.ClearSelection2(true);
            // Independent features preserve test centerline endpoints. One multi-group
            // weldment auto-trims later groups, which disconnected the first fixture.
            for(int i=0;i<lines.Length;i++) {
                var g=d.FeatureManager.CreateStructuralMemberGroup();g.Segments=new[]{lines[i]};g.ApplyCornerTreatment=false;g.GapWithinGroup=0;g.GapForOtherGroups=0;
                var feature=d.FeatureManager.InsertStructuralWeldment4(Path.Combine(Root,"profiles/1.000 x 0.065.SLDLFP"),(int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter,false,new DispatchWrapper[]{new(g)})??throw new Exception("Frame test construction failed");
                feature.Name="AcceptanceMember_"+(i+1);d.ClearSelection2(true);
            }
            d.EditRebuild3();if(Bodies(d).Length!=12||Bodies(d).Any(b=>b.Check3?.Count!=0))throw new Exception("Native frame body check failed");
            Mark(d,"RFR_TestOnly","Untrimmed beam acceptance fixture, not a manufactured chassis or eligible score");SaveNew(d,output);
            var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;var study=mgr.CreateNewStudy3("TranslationOnlyFrameAcceptance",0,0,out int e);
            if(e!=0||study==null)throw new Exception("Study "+e);mgr.ActiveStudy=mgr.StudyCount-1;var bm=study.BeamManager;
            if(bm.BeamCount!=12)throw new Exception("Wrong beam count: "+bm.BeamCount);
            string library=Path.Combine(App.GetExecutablePath(),"lang/english/sldmaterials/solidworks materials.sldmat");
            for(int i=0;i<12;i++)if(!bm.GetBeamBodyAt(i,out e).SetLibraryMaterial2(library,"Plain Carbon Steel"))throw new Exception("Beam material failed");
            var joints=bm.GetJointGroup(out e);joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.IncludeTreatAsJointForClearanceLessThan=true;
            joints.PinBallRadiusUnit=0;joints.PinBallRadius=.5;joints.CalculateJoints();e=joints.JointsEndEdit();if(e!=0)throw new Exception("Joints "+e);
            Save(d);var bound=BindBeamJoints(d,study,points);var loads=study.LoadsAndRestraintsManager;
            var fixture=loads.AddRestraint((int)swsRestraintType_e.swsRestraintTypeImmovable,bound.Take(4).Select(j=>j.entity).ToArray(),null,out int fixtureError);
            if(fixture==null||fixtureError!=0)throw new Exception("Translation-only fixture "+fixtureError);
            object direction=DirectionPlane(d,"Front Plane",out object planeTransform);var loadData=new List<object>();
            for(int i=4;i<8;i++) {
                double forceN=Math.Sign(points[i][1])*100*scale;
                var force=loads.AddForce3(0,1,2,0,0,0,Array.Empty<double>(),Array.Empty<double>(),false,false,0,0,4,forceN,new double[]{1,1,forceN,1,1,1},false,false,new[]{bound[i].entity},direction,false,out e);
                if(force==null||e!=0)throw new Exception("Force "+e);force.ForceBeginEdit();force.Unit=0;e=force.ForceEndEdit();if(e!=0)throw new Exception("Force edit "+e);
                loadData.Add(new {pointM=points[i],forceN=new[]{0d,0,forceN},joint=bound[i].nativeName});
            }
            study.StaticStudyOptions.ResultFolder=Path.GetDirectoryName(output);study.StaticStudyOptions.UseSoftSpring2=false;study.StaticStudyOptions.UseInertialRelief2=false;
            study.Mesh.GetDefaultElementSizeAndTolerance(2,out double size,out double tol);e=study.CreateMesh(2,size,tol);if(e!=0)throw new Exception("Mesh "+e);Save(d);
            Stage("Solving translation-only restraint frame acceptance");int solve=study.RunAnalysis();if(solve!=0)throw new Exception("Solve "+solve);
            var rotation=study.Results.GetRotationalDisplacement(1,null,3,out int re);var reaction=study.Results.GetReactionForcesAndMoments(1,null,0,out int fe);
            var translation=study.Results.GetTranslationalDisplacement(1,null,2,out int te);Save(d);
            var export=ExportStudyAudit(study,Path.GetDirectoryName(output)!);
            return new {path=output,cadSha256=FileSha(output),solve,beamCount=bm.BeamCount,loadScale=scale,units="m,N,Nm,rad",fixtureType=fixture.RestraintType,
                solverVersion=App.RevisionNumber(),configuration=study.ConfigurationName,softSprings=study.StaticStudyOptions.UseSoftSpring2,inertialRelief=study.StaticStudyOptions.UseInertialRelief2,
                rearFixturePointsM=points.Take(4),loads=loadData,planeTransform,pinball=new {requestedM=.0005,rawGetter=joints.PinBallRadius,unit=joints.PinBallRadiusUnit},
                binding=ReadBoundJointNodes(study.Mesh,bound),mesh=new {sizeM=size,toleranceM=tol,meshType=study.Mesh.MeshType,nodes=study.Mesh.GetNodes()},
                rotationError=re,reactionError=fe,translationError=te,rotation,reaction,translation,export,
                scope="API acceptance only: untrimmed twelve-beam test frame; no chassis score"};
        } finally {if(d.GetPathName()!="")App.CloseDoc(d.GetTitle());Open(restore);}
    }
}
