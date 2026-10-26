using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    static object BeamContextBinding(JsonElement job) {
        string source=Safe(job.GetProperty("source").GetString()!),output=Safe(job.GetProperty("output").GetString()!),restore=Safe(job.GetProperty("restore").GetString()!);
        if(!output.Contains(Path.Combine("cad","tests"))||File.Exists(output))throw new Exception("Fresh test copy required");
        Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(source,output);var d=Open(output);
        try {
            var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;var study=mgr.CreateNewStudy3("FixtureContextAcceptance",0,0,out int e);
            if(e!=0||study==null)throw new Exception("Study creation "+e);mgr.ActiveStudy=mgr.StudyCount-1;
            var bm=study.BeamManager;if(bm.BeamCount!=1)throw new Exception("Expected one beam");var beam=bm.GetBeamBodyAt(0,out e);
            string lib=Path.Combine(App.GetExecutablePath(),"lang/english/sldmaterials/solidworks materials.sldmat");if(!beam.SetLibraryMaterial2(lib,"Plain Carbon Steel"))throw new Exception("Material failed");
            var material=beam.GetBeamBodyMaterial();double shear=material.GetPropertyByName2(0,"GXY",out bool materialError);
            var joints=bm.GetJointGroup(out e);joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.CalculateJoints();e=joints.JointsEndEdit();
            if(e!=0)throw new Exception("Joints "+e);Save(d);
            var bound=BindBeamJoints(d,study,new[]{new[]{0d,0,0},new[]{1d,0,0}});
            d.ClearSelection2(true);var loads=study.LoadsAndRestraintsManager;
            var fixture=loads.AddRestraint(0,new object[]{bound[0].entity},null,out int fixtureError);
            if(fixture==null||fixtureError!=0)throw new Exception("Fixture "+fixtureError);
            Stage("Native joint fixture created");Save(d);
            object direction=DirectionPlane(d,"Right Plane",out object planeTransform);
            var force=loads.AddForce3(0,1,2,0,0,0,Array.Empty<double>(),Array.Empty<double>(),false,false,0,0,32,10,new double[]{1,1,1,1,1,10},false,false,new object[]{bound[1].entity},direction,false,out int forceError);
            if(force==null||forceError!=0)throw new Exception("Force "+forceError);force.ForceBeginEdit();force.Unit=0;e=force.ForceEndEdit();if(e!=0)throw new Exception("Force edit "+e);
            study.StaticStudyOptions.ResultFolder=Path.GetDirectoryName(output);study.StaticStudyOptions.UseSoftSpring2=false;study.StaticStudyOptions.UseInertialRelief2=false;
            study.Mesh.GetDefaultElementSizeAndTolerance(2,out double size,out double tol);int mesh=study.CreateMesh(2,size,tol);if(mesh!=0)throw new Exception("Mesh "+mesh);Save(d);
            var binding=ReadBoundJointNodes(study.Mesh,bound);
            Stage("Solving native beam torsion");int solve=study.RunAnalysis();if(solve!=0)throw new Exception("Solve "+solve);
            var rotation=study.Results.GetRotationalDisplacement(1,null,3,out int re);var reaction=study.Results.GetReactionForcesAndMoments(1,null,0,out int fe);Save(d);
            return new {path=output,cadSha256=FileSha(output),binding,planeTransform,fixtureError,forceError,solve,solved=true,units="m,N,Nm,rad",
                solverVersion=App.RevisionNumber(),configuration=study.ConfigurationName,softSprings=study.StaticStudyOptions.UseSoftSpring2,inertialRelief=study.StaticStudyOptions.UseInertialRelief2,
                material=new {name=material.MaterialName,shear,materialError},mesh=new {sizeM=size,toleranceM=tol,meshType=study.Mesh.MeshType,nodes=study.Mesh.GetNodes()},rotationError=re,reactionError=fe,rotation,reaction};
        } finally {App.CloseDoc(d.GetTitle());Open(restore);}
    }
}
