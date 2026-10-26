using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // The working frame is never altered by the study. The disposable study copy
    // deletes only the five reference bodies and retains all actual trimmed tubes.
    static object ChassisSimulation(JsonElement job) {
        string source=Safe(job.GetProperty("source").GetString()!),output=Safe(job.GetProperty("output").GetString()!),restore=Safe(job.GetProperty("restore").GetString()!);
        if(!output.Contains(Path.Combine("cad","tests"))||File.Exists(output))throw new Exception("Fresh study copy required");
        if(App.GetOpenDocumentByName(source) is ModelDoc2 live&&live.GetSaveFlag())throw new Exception("Unsaved candidate cannot be simulated");
        string sourceHash=FileSha(source);Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(source,output);
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        var fixturePoints=recipe.GetProperty("fixturePoints").EnumerateArray().Select(Numbers).ToArray();
        var loadPoints=recipe.GetProperty("loadPoints").EnumerateArray().Select(Numbers).ToArray();
        if(fixturePoints.Length!=8||loadPoints.Length!=8)throw new Exception("Prescribed eight fixture/eight load points required");
        double scale=job.TryGetProperty("loadScale",out var ls)?ls.GetDouble():1;
        double meshScale=job.TryGetProperty("meshScale",out var ms)?ms.GetDouble():1;
        if(Math.Abs(scale)!=1||meshScale<=0||meshScale>1)throw new Exception("Only prescribed/reversed load and default/refined mesh supported");
        var d=Open(output);
        try {
            if(!d.EditRebuild3())throw new Exception("Candidate rebuild failed");
            // Optional (diagnostic, for a VR3 package part): bodyNames maps the package's part-number
            // body names back to tube names in this disposable copy. The package has already removed
            // the reference solids, so with bodyNames an absent reference body is accepted.
            bool vr3=job.TryGetProperty("bodyNames",out var bn);var renamed=new List<string>();
            if(vr3) {
                var tubeOf=JsonDocument.Parse(File.ReadAllText(bn.GetString()!)).RootElement.EnumerateArray().ToDictionary(x=>x.GetProperty("partNumber").GetString()!,x=>x.GetProperty("tube").GetString()!);
                foreach(var b in Bodies(d))if(tubeOf.TryGetValue(b.Name,out var tube)){renamed.Add(b.Name+"->"+tube);b.Name=tube;}
                if(renamed.Count!=tubeOf.Count)throw new Exception($"Renamed {renamed.Count} of {tubeOf.Count} package bodies");
                if(!d.EditRebuild3())throw new Exception("Rebuild after body rename failed");
            }
            var excluded=recipe.GetProperty("referenceOnlyBodies").EnumerateArray().Select(x=>x.GetString()!).ToArray();
            var all=Bodies(d);var included=all.Where(b=>!excluded.Contains(b.Name)).ToArray();
            var before=included.ToDictionary(b=>b.Name,Volume);
            var toDelete=vr3?excluded.Where(n=>all.Any(b=>b.Name==n)).ToArray():excluded;
            if(toDelete.Length>0) {
                d.ClearSelection2(true);
                for(int i=0;i<toDelete.Length;i++)if(!all.Single(b=>b.Name==toDelete[i]).Select2(i>0,null))throw new Exception("Select reference body");
                var deleted=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Delete references failed");deleted.Name="Simulation_ExcludeReferenceSolids";
                d.ClearSelection2(true);if(!d.EditRebuild3()||deleted.GetErrorCode()!=0)throw new Exception("Exclude reference rebuild");
            }
            // The trims' temporary cutting surfaces would enter the study as shells.
            // Delete them from this disposable copy only (the cut features keep their result).
            var sheets=Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Cast<Body2>().ToArray();
            int surfaceCount=sheets.Length;
            if(surfaceCount>0) {
                d.ClearSelection2(true);
                for(int i=0;i<sheets.Length;i++)if(!sheets[i].Select2(i>0,null))throw new Exception("Select cutter surface "+sheets[i].Name);
                var delSurf=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Delete cutter surfaces failed");delSurf.Name="Simulation_ExcludeCutterSurfaces";
                d.ClearSelection2(true);if(!d.EditRebuild3()||delSurf.GetErrorCode()!=0)throw new Exception("Exclude cutter surfaces rebuild");
                if(Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Length!=0)throw new Exception("Cutter surfaces remain in study copy");
            }
            var tubes=Bodies(d);
            if(tubes.Length!=before.Count||tubes.Any(b=>!before.ContainsKey(b.Name)||Math.Abs(before[b.Name]-Volume(b))>1e-12||b.Check3?.Count!=0))throw new Exception("Structural body changed during exclusion");
            string db="";string materialName=((PartDoc)d).GetMaterialPropertyName2(d.ConfigurationManager.ActiveConfiguration.Name,out db);
            if(db.Equals("SOLIDWORKS Materials",StringComparison.OrdinalIgnoreCase))db=Path.Combine(App.GetExecutablePath(),"lang/english/sldmaterials/solidworks materials.sldmat");
            if(string.IsNullOrEmpty(materialName)||!File.Exists(db))throw new Exception("Candidate material database unavailable: "+db+" / "+materialName);
            Save(d);Stage("Creating native trimmed chassis study");
            var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;
            var study=mgr.CreateNewStudy3("PrescribedChassisTorsion",0,0,out int e);
            if(e!=0||study==null)throw new Exception("Study creation "+e);mgr.ActiveStudy=mgr.StudyCount-1;
            var bm=study.BeamManager;var materials=new List<object>();var convertedToBeam=new List<string>();
            if(bm.BeamCount<before.Count) {
                // TR procedure lesson (7/19): tubes the study meshes as solids must be
                // "Treated as Beam". Bodies whose cope used a Combine feature lose the
                // automatic structural-member classification.
                var sm=study.SolidManager;
                for(int c=0;c<sm.ComponentCount;c++) {
                    var comp=sm.GetComponentAt(c,out e);if(e!=0||comp==null)continue;
                    for(int k=0;k<comp.SolidBodyCount;k++) {
                        var sb=comp.GetSolidBodyAt(k,out e);if(e!=0||sb==null)continue;
                        string sbName=sb.SolidBodyName;
                        // Returns nonzero for bodies that are already beams; the final
                        // beam-count check below is the gate.
                        if(sb.ConvertToBeamBody()==0){convertedToBeam.Add(sbName);Stage("Treated as beam: "+sbName);}
                    }
                }
                bm=study.BeamManager;
            }
            if(bm.BeamCount!=before.Count)throw new Exception("Not every structural body is a beam: "+bm.BeamCount+" vs "+before.Count+"; converted "+string.Join(",",convertedToBeam));
            for(int i=0;i<bm.BeamCount;i++) {
                var beam=bm.GetBeamBodyAt(i,out e);if(e!=0||beam==null||!beam.SetLibraryMaterial2(db,materialName))throw new Exception("Beam material "+i);
                var mat=beam.GetBeamBodyMaterial();double young=mat.GetPropertyByName2(0,"EX",out bool exError),shear=mat.GetPropertyByName2(0,"GXY",out bool gError),density=mat.GetPropertyByName2(0,"DENS",out bool rhoError);
                materials.Add(new {beam=beam.BeamBodyName,name=mat.MaterialName,youngPa=young,shearPa=shear,densityKgM3=density,exError,gError,rhoError});
                if(exError||gError||rhoError||young<=0||shear<=0||density<=0)throw new Exception("Native beam material readback invalid");
            }
            var joints=bm.GetJointGroup(out e);joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.IncludeTreatAsJointForClearanceLessThan=true;
            joints.PinBallRadiusUnit=0;joints.PinBallRadius=1.27;joints.CalculateJoints();e=joints.JointsEndEdit();if(e!=0)throw new Exception("Joint calculation "+e);
            Save(d);var bindPoints=fixturePoints.Concat(loadPoints).ToArray();
            var members=recipe.GetProperty("members").EnumerateArray().Select(m=>(a:Numbers(m.GetProperty("a")),b:Numbers(m.GetProperty("b")))).ToArray();
            var dirs=bindPoints.Select(p=>members.SelectMany(m=>{
                var r=new List<double[]>();
                if(Norm(Sub(m.a,p))<1e-6)r.Add(Unit(Sub(m.b,m.a)));
                if(Norm(Sub(m.b,p))<1e-6)r.Add(Unit(Sub(m.a,m.b)));
                return r;}).ToArray()).ToArray();
            var preferSide=job.TryGetProperty("preferSideView",out var ps)?ps.EnumerateArray().Select(x=>x.GetInt32()).ToHashSet():new HashSet<int>();
            var bound=BindBeamJoints(d,study,bindPoints,dirs,preferSide,locateReach:.01);var loads=study.LoadsAndRestraintsManager;
            var fixture=loads.AddRestraint((int)swsRestraintType_e.swsRestraintTypeImmovable,bound.Take(8).Select(j=>j.entity).ToArray(),null,out e);
            if(fixture==null||e!=0)throw new Exception("Translation-only rear fixture "+e);
            // TR procedure: forces "normal to plane" referenced to the ground plane.
            string planeName=job.TryGetProperty("loadPlane",out var lp)?lp.GetString()!:"Ground Plane";
            object direction=DirectionPlane(d,planeName,out object planeTransform);var loadData=new List<object>();
            for(int i=0;i<8;i++) {
                double fz=Math.Sign(loadPoints[i][1])*100*4.4482216152605*scale;
                var force=loads.AddForce3(0,1,2,0,0,0,Array.Empty<double>(),Array.Empty<double>(),false,false,0,0,4,fz,new[]{1d,1,fz,1,1,1},false,false,new[]{bound[i+8].entity},direction,false,out e);
                if(force==null||e!=0)throw new Exception("Force "+e);force.ForceBeginEdit();force.Unit=0;e=force.ForceEndEdit();if(e!=0)throw new Exception("Force edit "+e);
                loadData.Add(new {pointM=loadPoints[i],forceN=new[]{0d,0,fz},joint=bound[i+8].nativeName});
            }
            study.StaticStudyOptions.ResultFolder=Path.GetDirectoryName(output);study.StaticStudyOptions.UseSoftSpring2=false;study.StaticStudyOptions.UseInertialRelief2=false;
            study.Mesh.GetDefaultElementSizeAndTolerance(2,out double size,out double tol);e=study.CreateMesh(2,size*meshScale,tol*meshScale);if(e!=0)throw new Exception("Mesh "+e);Save(d);
            var export=ExportStudyAudit(study,Path.GetDirectoryName(output)!);
            Stage("Solving native trimmed chassis torsion");int solve=study.RunAnalysis();if(solve!=0)throw new Exception("Native chassis solve "+solve);
            var rotation=study.Results.GetRotationalDisplacement(1,null,3,out int re);var reaction=study.Results.GetReactionForcesAndMoments(1,null,0,out int fe);var translation=study.Results.GetTranslationalDisplacement(1,null,2,out int te);Save(d);
            return new {path=output,cadSha256=FileSha(output),source,renamedPackageBodies=renamed,convertedToBeam,excludedCutterSurfaces=surfaceCount,sourceCadSha256=sourceHash,solve,beamCount=bm.BeamCount,includedBodies=before.Keys,excludedBodies=excluded,materials,materialDatabase=db,materialDatabaseSha256=FileSha(db),materialName,
                loadScale=scale,loadPlane=planeName,units="m,N,Nm,rad",fixtureType=fixture.RestraintType,solverVersion=App.RevisionNumber(),configuration=study.ConfigurationName,
                softSprings=study.StaticStudyOptions.UseSoftSpring2,inertialRelief=study.StaticStudyOptions.UseInertialRelief2,rearFixturePointsM=fixturePoints,loads=loadData,planeTransform,
                probes=recipe.GetProperty("probes").Clone(),pinball=new {requestedM=.00127,rawGetter=joints.PinBallRadius,unit=joints.PinBallRadiusUnit},binding=ReadBoundJointNodes(study.Mesh,bound),
                mesh=new {defaultSizeM=size,defaultToleranceM=tol,meshScale,sizeM=size*meshScale,toleranceM=tol*meshScale,meshType=study.Mesh.MeshType,nodes=study.Mesh.GetNodes()},
                rotationError=re,reactionError=fe,translationError=te,rotation,reaction,translation,export,
                scope="Native trimmed chassis result; eligible only after independent geometry, rules, mapping, equilibrium and mesh checks"};
        } finally {App.CloseDoc(d.GetTitle());if(File.Exists(restore))Open(restore);}
    }
}
