using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // Diagnostic only: which beams does SOLIDWORKS join automatically? Copies a part into
    // cad/tests, sets up the beam study the same way as chassis-simulation (reference and
    // cutter bodies removed, every tube a beam, prescribed material), calculates joints at
    // each requested clearance, meshes and exports the Nastran deck. No fixtures, loads or
    // solve, so it never produces a score; connectivity is read from the exported deck.
    static object JointDiagnostics(JsonElement job) {
        string source=Safe(job.GetProperty("source").GetString()!),output=Safe(job.GetProperty("output").GetString()!);
        string? restore=job.TryGetProperty("restore",out var r)?Safe(r.GetString()!):null;
        if(!output.Contains(Path.Combine("cad","tests"))||File.Exists(output))throw new Exception("Fresh test copy required");
        if(App.GetOpenDocumentByName(source) is ModelDoc2 live&&live.GetSaveFlag())throw new Exception("Unsaved source cannot be diagnosed");
        var clearances=job.TryGetProperty("clearancesMm",out var cs)?cs.EnumerateArray().Select(x=>x.GetDouble()).ToArray():new[]{1.27};
        if(clearances.Length==0||clearances.Any(c=>c<=0||c>50))throw new Exception("Clearances must be in (0, 50] mm");
        var excluded=job.TryGetProperty("recipe",out var rp)
            ?JsonDocument.Parse(File.ReadAllText(rp.GetString()!)).RootElement.GetProperty("referenceOnlyBodies").EnumerateArray().Select(x=>x.GetString()!).ToArray()
            :Array.Empty<string>();
        string sourceHash=FileSha(source);Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(source,output);
        var d=Open(output);
        try {
            if(!d.EditRebuild3())throw new Exception("Rebuild failed");
            var present=Bodies(d).Where(b=>excluded.Contains(b.Name)).ToArray();
            if(present.Length>0) {
                d.ClearSelection2(true);
                for(int i=0;i<present.Length;i++)if(!present[i].Select2(i>0,null))throw new Exception("Select reference body");
                var del=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Delete references failed");del.Name="Diagnostics_ExcludeReferenceSolids";
                d.ClearSelection2(true);if(!d.EditRebuild3()||del.GetErrorCode()!=0)throw new Exception("Exclude reference rebuild");
            }
            var sheets=Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Cast<Body2>().ToArray();
            if(sheets.Length>0) {
                d.ClearSelection2(true);
                for(int i=0;i<sheets.Length;i++)if(!sheets[i].Select2(i>0,null))throw new Exception("Select cutter surface");
                var del=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Delete cutter surfaces failed");del.Name="Diagnostics_ExcludeCutterSurfaces";
                d.ClearSelection2(true);if(!d.EditRebuild3()||del.GetErrorCode()!=0)throw new Exception("Exclude cutter surfaces rebuild");
            }
            var tubes=Bodies(d).Select(b=>b.Name).ToArray();
            string library=Path.Combine(App.GetExecutablePath(),"lang/english/sldmaterials/solidworks materials.sldmat");
            string materialName=job.TryGetProperty("material",out var mn)?mn.GetString()!:"AISI 4130 Steel, normalized at 870C";
            Save(d);
            var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;
            var study=mgr.CreateNewStudy3("JointDiagnostics",0,0,out int e);
            if(e!=0||study==null)throw new Exception("Study creation "+e);mgr.ActiveStudy=mgr.StudyCount-1;
            var bm=study.BeamManager;var converted=new List<string>();
            if(bm.BeamCount<tubes.Length) {
                var sm=study.SolidManager;
                for(int c=0;c<sm.ComponentCount;c++) {
                    var comp=sm.GetComponentAt(c,out e);if(e!=0||comp==null)continue;
                    for(int k=0;k<comp.SolidBodyCount;k++){var sb=comp.GetSolidBodyAt(k,out e);if(e==0&&sb!=null&&sb.ConvertToBeamBody()==0)converted.Add(sb.SolidBodyName);}
                }
                bm=study.BeamManager;
            }
            if(bm.BeamCount!=tubes.Length)throw new Exception($"Not every body is a beam: {bm.BeamCount} vs {tubes.Length}");
            var beams=new List<object>();
            for(int i=0;i<bm.BeamCount;i++) {
                var beam=bm.GetBeamBodyAt(i,out e);if(e!=0||beam==null||!beam.SetLibraryMaterial2(library,materialName))throw new Exception("Beam material "+i);
                beams.Add(new {index=i+1,name=beam.BeamBodyName,type=beam.BeamType,end1=beam.BeamEnd1ConnectionType,end2=beam.BeamEnd2ConnectionType});
            }
            var runs=new List<object>();
            foreach(double c in clearances) {
                var joints=bm.GetJointGroup(out e);joints.JointsBeginEdit();joints.IncludeAllSelectedBeam=true;joints.IncludeTreatAsJointForClearanceLessThan=true;
                joints.PinBallRadiusUnit=0;joints.PinBallRadius=c;joints.CalculateJoints();e=joints.JointsEndEdit();if(e!=0)throw new Exception("Joint calculation "+e);
                study.Mesh.GetDefaultElementSizeAndTolerance(2,out double size,out double tol);e=study.CreateMesh(2,size,tol);if(e!=0)throw new Exception("Mesh "+e);
                string target=Path.Combine(Path.GetDirectoryName(output)!,$"deck-{c.ToString("0.###",System.Globalization.CultureInfo.InvariantCulture)}mm");Directory.CreateDirectory(target);
                int code=study.ExportSimulationStudy(target,"audit",(int)swsNastranExportOption_e.swsNastranExportOption_LongFree,0,0,
                    (int)swsStudyExportOption_e.swsStudyExportOption_Nastran,(int)swsNastranExportUnit_e.swsNastranExportUnit_SI);
                runs.Add(new {clearanceMm=c,pinballRaw=joints.PinBallRadius,pinballUnit=joints.PinBallRadiusUnit,meshSizeM=size,exportCode=code,files=Directory.GetFiles(target).Select(p=>new {path=p,sha256=FileSha(p)}).ToArray()});
                Save(d);
            }
            return new {path=output,source,sourceCadSha256=sourceHash,cadSha256=FileSha(output),excludedBodies=present.Select(b=>b.Name).ToArray(),excludedCutterSurfaces=sheets.Length,
                converted,beams,runs,scope="Diagnostic joint/deck export only; no fixtures, loads or solve"};
        } finally {App.CloseDoc(d.GetTitle());if(restore!=null&&File.Exists(restore))Open(restore);}
    }
}
