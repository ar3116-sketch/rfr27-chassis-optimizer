using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // Read-only audit of an existing, already solved study (for example one set up by hand in
    // the SOLIDWORKS UI): export its Nastran deck and read back its solved rotations and mesh
    // nodes. It does not edit, re-mesh or re-solve the study.
    static object ExportStudy(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!),output=Safe(job.GetProperty("outputDir").GetString()!);
        if(!output.Contains(Path.Combine("cad","tests"))||(Directory.Exists(output)&&Directory.EnumerateFileSystemEntries(output).Any()))throw new Exception("Fresh cad/tests output directory required");
        string name=job.GetProperty("study").GetString()!;
        var d=Open(path);Activate(d);
        var mgr=RequireSimulation().ActiveDoc.StudyManager;int idx=-1;
        for(int i=0;i<mgr.StudyCount;i++)if(mgr.GetStudy(i).Name==name)idx=i;
        if(idx<0)throw new Exception("Study not found: "+name);
        mgr.ActiveStudy=idx;var study=mgr.GetStudy(idx);
        Directory.CreateDirectory(output);
        int code=study.ExportSimulationStudy(output,"audit",(int)swsNastranExportOption_e.swsNastranExportOption_LongFree,0,0,
            (int)swsStudyExportOption_e.swsStudyExportOption_Nastran,(int)swsNastranExportUnit_e.swsNastranExportUnit_SI);
        var results=study.Results??throw new Exception("Study has no results");
        var rotation=results.GetRotationalDisplacement(1,null,3,out int re);
        var bm=study.BeamManager;var beams=new List<string>();
        for(int i=0;i<bm.BeamCount;i++){var b=bm.GetBeamBodyAt(i,out int e);beams.Add(b?.BeamBodyName??"");}
        return new {path,cadSha256=FileSha(path),study=name,exportCode=code,files=Directory.GetFiles(output).Select(p=>new {path=p,sha256=FileSha(p)}).ToArray(),
            beams,rotationError=re,rotation,mesh=new {nodes=study.Mesh.GetNodes()},scope="Read-only export of an existing study; no edit, mesh or solve"};
    }
}
