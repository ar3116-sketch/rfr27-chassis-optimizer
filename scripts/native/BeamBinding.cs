using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // Isolated experiment: access real end faces directly, never pick manipulators.
    static object BeamFaceBinding(JsonElement job) {
        string source=Safe(job.GetProperty("source").GetString()!);
        string output=Safe(job.GetProperty("output").GetString()!);
        string restore=Safe(job.GetProperty("restore").GetString()!);
        if(!output.Contains(Path.Combine("cad","tests"))||File.Exists(output))throw new Exception("New test path required");
        Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(source,output);
        var d=Open(output);
        try {
            var body=Bodies(d).Single();
            var faces=Arr(body.GetFaces()).Cast<Face2>().Where(f=>((Surface)f.GetSurface()).IsPlane()).ToArray();
            var root=faces.Single(f=>((double[])f.GetBox()).Where((v,i)=>i==0||i==3).All(v=>Math.Abs(v)<1e-7));
            var tip=faces.Single(f=>((double[])f.GetBox()).Where((v,i)=>i==0||i==3).All(v=>Math.Abs(v-1)<1e-7));
            var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;
            var study=mgr.CreateNewStudy3("DirectFaceBinding",0,0,out int e);
            if(e!=0||study==null)throw new Exception("Study creation failed "+e);
            mgr.ActiveStudy=mgr.StudyCount-1;
            if(study.BeamManager.BeamCount!=1)throw new Exception("Expected one native beam");
            var beam=study.BeamManager.GetBeamBodyAt(0,out e);
            string lib=Path.Combine(App.GetExecutablePath(),"lang/english/sldmaterials/solidworks materials.sldmat");
            if(!beam.SetLibraryMaterial2(lib,"Plain Carbon Steel"))throw new Exception("Beam material failed");
            var joints=study.BeamManager.GetJointGroup(out e);
            joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.CalculateJoints();e=joints.JointsEndEdit();
            if(e!=0)throw new Exception("Joint generation failed "+e);
            Stage("Applying fixed restraint directly to beam end face");
            var fix=study.LoadsAndRestraintsManager.AddRestraint(0,new object[]{root},null,out int fixtureError);
            if(fixtureError!=0||fix==null){Save(d);return new {path=output,fixtureError,fixtureCreated=fix!=null,solved=false,selectionCalls=0};}
            Stage("Applying moment directly to beam end face");
            var force=study.LoadsAndRestraintsManager.AddForce3(0,0,-1,0,0,0,new double[0],new double[0],false,false,0,0,8,10,new double[]{1,1,1,10,1,1},false,false,new object[]{tip},null,false,out int forceError);
            if(forceError!=0||force==null){Save(d);return new {path=output,fixtureError,fixtureCreated=true,forceError,solved=false,selectionCalls=0};}
            force.ForceBeginEdit();force.Unit=0;int forceEdit=force.ForceEndEdit();if(forceEdit!=0)throw new Exception("Force edit "+forceEdit);
            study.StaticStudyOptions.ResultFolder=Path.GetDirectoryName(output);study.StaticStudyOptions.UseSoftSpring2=false;study.StaticStudyOptions.UseInertialRelief2=false;
            study.Mesh.GetDefaultElementSizeAndTolerance(0,out double size,out double tol);
            int mesh=study.CreateMesh(0,size,tol);Save(d);if(mesh!=0)throw new Exception("Mesh failed "+mesh);
            Stage("Solving direct-face beam test");int solve=study.RunAnalysis();
            object? rx=null,reaction=null;int rotationError=-1,reactionError=-1;
            if(solve==0){rx=study.Results.GetRotationalDisplacement(1,null,3,out rotationError);reaction=study.Results.GetReactionForcesAndMoments(1,null,0,out reactionError);}
            Save(d);
            return new {path=output,fixtureError,forceError,mesh,meshType=study.Mesh.MeshType,solve,rotationError,reactionError,rx,reaction,nodes=study.Mesh.GetNodes(),selectionCalls=0};
        } finally {App.CloseDoc(d.GetTitle());Open(restore);}
    }
}
