using System.Text.Json;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    static void Stage(string value) => Console.Error.WriteLine($"{DateTime.UtcNow:O} {value}");
    static ICosmosWorks RequireSimulation() {
        string prog="SldWorks.Simulation."+(int.Parse(App.RevisionNumber().Split('.')[0])-15);
        ICwAddincallback? add=App.GetAddInObject(prog) as ICwAddincallback;
        if(add==null){
            string dll=Path.Combine(App.GetExecutablePath(),"Simulation","cosworks.dll");
            int load=App.LoadAddIn(dll);Stage($"Simulation LoadAddIn returned {load}");
            add=App.GetAddInObject(prog) as ICwAddincallback;
        }
        return add?.CosmosWorks??throw new InvalidOperationException("Native SolidWorks Simulation API unavailable. Run scripts/diagnose-simulation.ps1 and inspect installed full Simulation features. No surrogate score will be emitted.");
    }
    static object JointSelectionTest(JsonElement job) => throw new NotSupportedException("Experimental beam-joint picking is disabled: SelectByID2 enters interactive manipulation and API cancellation timed out. Full Simulation is installed and the native solid solve passed. See handoff/evidence/experimental-beam-selector-20261011-1537.cs and WORK_LOG.md.");
    static object SelectionDiagnostics(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);
        App.GetRunningCommandInfo(out int command,out string commandText,out bool commandIsUser);
        var sm=(SelectionMgr)d.SelectionManager;var selected=new List<object>();
        for(int i=1;i<=sm.GetSelectedObjectCount2(-1);i++){
            object obj=sm.GetSelectedObject6(i,-1);var man=obj as IManipulator;object? specific=man?.GetSpecificManipulator();
            selected.Add(new {type=sm.GetSelectedObjectType3(i,-1),name=man?.Name,triadOrigin=(specific as ITriadManipulator)?.Origin?.ArrayData,arrowOrigin=(specific as IDragArrowManipulator)?.Origin?.ArrayData});
        }
        string? export=null;
        if(job.TryGetProperty("journalOutput",out var p)){export=Safe(p.GetString()!);if(File.Exists(export))throw new Exception("Refusing overwrite");App.JournalManager.WriteJournalToMacro(export);}
        return new {path=d.GetPathName(),command,commandText,commandIsUser,commandInProgress=App.CommandInProgress,selected,journalExport=export,exportExists=export!=null&&File.Exists(export)};
    }
    static object SimulationBeamSolve(JsonElement job) => BeamContextBinding(job);
    static object SimulationBeamTest(JsonElement job) {
        string path=Safe(job.GetProperty("output").GetString()!);if(File.Exists(path)||!path.Contains(Path.Combine("cad","tests")))throw new Exception("New test path required");
        var d=New(false);d.FeatureManager.InsertWeldmentFeature();d.SketchManager.Insert3DSketch(true);
        var line=d.SketchManager.CreateLine(0,0,0,1,0,0);d.SketchManager.Insert3DSketch(true);
        var group=d.FeatureManager.CreateStructuralMemberGroup();group.Segments=new SketchSegment[]{line};group.ApplyCornerTreatment=false;group.Angle=0;group.GapWithinGroup=0;group.GapForOtherGroups=0;group.MirrorProfile=false;d.ClearSelection2(true);
        var f=d.FeatureManager.InsertStructuralWeldment4(Path.Combine(Root,"profiles","1.000 x 0.065.sldlfp"),(int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter,false,new DispatchWrapper[]{new(group)})??throw new Exception("Tube creation failed");
        f.Name="AcceptanceTube";d.EditRebuild3();SaveNew(d,path);
        var cw=RequireSimulation();
        var study=cw.ActiveDoc.StudyManager.CreateNewStudy3("NativeBeamAcceptance",(int)swsAnalysisStudyType_e.swsAnalysisStudyTypeStatic,0,out int e);
        if(e!=0||study==null)throw new Exception($"Study creation {e}");
        var bm=study.BeamManager;if(bm.BeamCount!=1)throw new Exception($"Expected one beam: {bm.BeamCount}");
        var b=bm.GetBeamBodyAt(0,out e);string library=Path.Combine(App.GetExecutablePath(),"lang","english","sldmaterials","solidworks materials.sldmat");
        if(!b.SetLibraryMaterial2(library,"Plain Carbon Steel"))throw new Exception("Beam material failed");
        var joints=bm.GetJointGroup(out e);joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.IncludeTreatAsJointForClearanceLessThan=true;joints.PinBallRadiusUnit=(int)swsPinballUnit_e.swsPinballmm;joints.PinBallRadius=.5;joints.CalculateJoints();e=joints.JointsEndEdit();if(e!=0)throw new Exception($"Joint calculation {e}");
        study.Mesh.GetDefaultElementSizeAndTolerance((int)swsLinearUnit_e.swsLinearUnitMeters,out double size,out double tol);
        int mesh=study.CreateMesh((int)swsLinearUnit_e.swsLinearUnitMeters,size,tol);
        d.ClearSelection2(true);Save(d);d.ShowNamedView2("*Isometric",7);d.ViewZoomtofit2();d.SaveBMP(Path.ChangeExtension(path,"bmp"),1400,900);
        var result=new {path,beamCount=bm.BeamCount,beamName=b.BeamBodyName,jointRadiusRaw=joints.PinBallRadius,jointRadiusUnit=joints.PinBallRadiusUnit,meshCode=mesh,meshType=study.Mesh.MeshType,nodes=study.Mesh.GetNodes(),jointSelectionVerified=false,solved=false};
        if(job.TryGetProperty("restore",out var restore)){Save(d);App.CloseDoc(d.GetTitle());Open(restore.GetString()!);}
        return result;
    }
    static object SimulationSolidTest(JsonElement job) {
        string path=Safe(job.GetProperty("output").GetString()!);
        if(!path.Contains(Path.Combine("cad","tests"))||File.Exists(path))throw new Exception("New test path required");
        var d=New(false);
        if(!d.Extension.SelectByID2("Front Plane","PLANE",0,0,0,false,0,null,0))throw new Exception("Plane selection failed");
        d.SketchManager.InsertSketch(true);d.SketchManager.CreateCornerRectangle(-.01,-.01,0,.01,.01,0);
        var f=d.FeatureManager.FeatureExtrusion2(true,false,false,0,0,.1,0,false,false,false,false,0,0,false,false,false,false,true,true,true,0,0,false)??throw new Exception("Extrusion failed");
        f.Name="AxialTestBar";d.EditRebuild3();SaveNew(d,path);
        var body=Bodies(d).Single(); if(body.Check3?.Count>0)throw new Exception("Invalid bar");
        var faces=Arr(body.GetFaces()).Cast<Face2>().Where(x=>((Surface)x.GetSurface()).IsPlane()).ToArray();
        var root=faces.Single(x=>Math.Abs(((double[])x.GetBox())[2])<1e-8&&Math.Abs(((double[])x.GetBox())[5])<1e-8);
        var tip=faces.Single(x=>Math.Abs(((double[])x.GetBox())[2]-.1)<1e-8&&Math.Abs(((double[])x.GetBox())[5]-.1)<1e-8);
        var cw=RequireSimulation();
        var mgr=cw.ActiveDoc.StudyManager;var study=mgr.CreateNewStudy3("NativeAxialAcceptance",(int)swsAnalysisStudyType_e.swsAnalysisStudyTypeStatic,0,out int e);
        if(e!=0||study==null)throw new Exception($"Study creation {e}");
        var sb=study.SolidManager.GetComponentAt(0,out e).GetSolidBodyAt(0,out e);
        string library=Path.Combine(App.GetExecutablePath(),"lang","english","sldmaterials","solidworks materials.sldmat");
        if(!sb.SetLibraryMaterial2(library,"Plain Carbon Steel"))throw new Exception("Material assignment failed");
        var material=sb.GetSolidBodyMaterial();bool materialError=false;double elastic=material.GetPropertyByName2(0,"EX",out materialError);
        var l=study.LoadsAndRestraintsManager;
        var fix=l.AddRestraint((int)swsRestraintType_e.swsRestraintTypeFixed,new object[]{root},null,out e);
        if(fix==null||e!=0)throw new Exception($"Fixture failed {e}");
        var force=l.AddForce3((int)swsForceType_e.swsForceTypeNormal,(int)swsSelectionType_e.swsSelectionFaceEdgeVertexPoint,-1,0,0,0,new double[0],new double[0],false,false,0,0,0,100,new double[]{1,1,1,1,1,1},false,false,new object[]{tip},null,false,out e);
        if(force==null||e!=0)throw new Exception($"Force failed {e}");
        force.ForceBeginEdit();force.Unit=(int)swsForceUnit_e.swsForceUnitNOrNm;force.NormalForceOrTorqueValue=100;
        if(force.ForceEndEdit()!=0)throw new Exception("Force edit failed");
        study.StaticStudyOptions.ResultFolder=Path.GetDirectoryName(path);
        study.StaticStudyOptions.UseSoftSpring2=false;study.StaticStudyOptions.UseInertialRelief2=false;
        study.Mesh.GetDefaultElementSizeAndTolerance((int)swsLinearUnit_e.swsLinearUnitMeters,out double size,out double tol);
        e=study.CreateMesh((int)swsLinearUnit_e.swsLinearUnitMeters,size,tol);if(e!=0)throw new Exception($"Mesh failed {e}");
        Save(d);int solve=study.RunAnalysis();if(solve!=0)throw new Exception($"Native analysis failed {solve}");
        var results=study.Results;
        object translation=results.GetTranslationalDisplacement(1,null,(int)swsLinearUnit_e.swsLinearUnitMeters,out int displacementError);
        object reaction=results.GetReactionForcesAndMoments(1,null,(int)swsForceUnit_e.swsForceUnitNOrNm,out int reactionError);
        if(displacementError!=0||reactionError!=0)throw new Exception($"Results failed {displacementError}, {reactionError}");
        Save(d);
        return new {path,study="NativeAxialAcceptance",licenseVerifiedBySolve=true,solveCode=solve,
            units="m,N,Pa",testGeometry=new {lengthM=.1,areaM2=.0004,forceN=100},
            material=new {name=material.MaterialName,elasticModulus=elastic,materialError,units=material.MaterialUnits,library},
            mesh=new {size,tol,nodes=study.Mesh.NodeCount,elements=study.Mesh.ElementCount,state=study.Mesh.MeshState},translation,reaction,
            scope="Solid-element axial acceptance only; beam joints and chassis study are separate checks"};
    }
}
