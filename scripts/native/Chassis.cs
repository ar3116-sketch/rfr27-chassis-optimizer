using System.Runtime.InteropServices;
using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    static double[] Numbers(JsonElement e)=>e.EnumerateArray().Select(v=>v.GetDouble()).ToArray();
    static Feature? Find(ModelDoc2 d,string name){for(Feature? f=d.IFirstFeature();f!=null;f=f.IGetNextFeature())if(f.Name==name)return f;return null;}
    static void Mark(ModelDoc2 doc,string key,string value)=>doc.Extension.CustomPropertyManager[""].Add3(key,30,value,1);
    static void Save(ModelDoc2 doc){int e=0,w=0;if(!doc.Save3(1,ref e,ref w)||e!=0)throw new Exception($"Save3 {e}, {w}");}
    static object PoseAssembly(JsonElement job) {
        string output=Safe(job.GetProperty("output").GetString()!);
        if(File.Exists(output))return VerifyPose(job);
        var asm=New(true); var mu=(MathUtility)App.GetMathUtility(); var expected=new List<object>();
        foreach(var part in job.GetProperty("components").EnumerateArray()) {
            string path=Safe(part.GetProperty("path").GetString()!); Open(path); Activate(asm);
            var c=((AssemblyDoc)asm).AddComponent5(path,0,"",false,"",0,0,0) as Component2??throw new Exception("Insert failed");
            c.Name2=part.GetProperty("name").GetString()!;
            var m=Numbers(part.GetProperty("transform")); if(m.Length!=16||m.Any(x=>!double.IsFinite(x)))throw new Exception("Invalid matrix");
            c.Transform2=(MathTransform)mu.CreateTransform(m);
            if(part.TryGetProperty("configuration",out var config))c.ReferencedConfiguration=config.GetString();
            asm.ClearSelection2(true);if(!c.Select4(false,null,false))throw new Exception("Select failed");((AssemblyDoc)asm).FixComponent();
            expected.Add(new {name=c.Name2,path,transform=m});
        }
        Mark(asm,"RFR_Status","DESIGN REFERENCE — proposed pose; not driver clearance approval");
        Mark(asm,"RFR_PoseDefinition",job.GetProperty("poseDefinition").GetString()!);
        asm.ClearSelection2(true); if(!asm.EditRebuild3())throw new Exception("Assembly rebuild failed"); SaveNew(asm,output);App.CloseDoc(asm.GetTitle());
        return VerifyPose(job);
    }
    static object VerifyPose(JsonElement job) {
        string output=Safe(job.GetProperty("output").GetString()!);
        var d=Open(output); var inspection=Inspect(d);
        var actual=Arr(((AssemblyDoc)d).GetComponents(true)).Cast<Component2>().ToArray();
        if(actual.Length!=job.GetProperty("components").GetArrayLength())throw new Exception("Component count mismatch after reopen");
        foreach(var part in job.GetProperty("components").EnumerateArray()) {
            var c=actual.Single(x=>string.Equals(x.GetPathName(),part.GetProperty("path").GetString(),StringComparison.OrdinalIgnoreCase));
            var requested=Numbers(part.GetProperty("transform"));var found=(double[])c.Transform2.ArrayData;
            if(found.Zip(requested,(a,b)=>Math.Abs(a-b)).Max()>1e-9)throw new Exception("Saved transform mismatch: "+c.Name2);
        }
        ChassisView(d);Save(d);string preview=Path.ChangeExtension(output,"bmp");d.SaveBMP(preview,1600,1000);
        return new {output,reopened=inspection,preview,transformsVerified=true};
    }
    static object BuildHoops(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);var records=new List<object>();
        if(d.GetType()!=1)throw new Exception("Part required");
        if(!job.GetProperty("path").GetString()!.Contains("candidate",StringComparison.OrdinalIgnoreCase))throw new Exception("Explicit candidate copy required");
        d.FeatureManager.InsertWeldmentFeature();
        foreach(var h in job.GetProperty("hoops").EnumerateArray()) {
            string name=h.GetProperty("name").GetString()!;
            if(Find(d,name)!=null)throw new Exception("Feature already exists");
            var f=Find(d,h.GetProperty("sketch").GetString()!)??throw new Exception("Missing hoop sketch");
            var s=(Sketch)f.GetSpecificFeature2();var all=Arr(s.GetSketchSegments()).Cast<SketchSegment>().ToArray();
            var segments=h.GetProperty("segments").EnumerateArray().Select(i=>all[i.GetInt32()]).ToArray();
            if(segments.Any(x=>x.ConstructionGeometry))throw new Exception("Construction segment selected");
            string profile=Path.GetFullPath(Path.Combine(Root,"profiles",h.GetProperty("profile").GetString()!));
            if(!profile.StartsWith(Path.Combine(Root,"profiles")+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))throw new Exception("Invalid profile path");
            var group=d.FeatureManager.CreateStructuralMemberGroup();group.Segments=segments;group.ApplyCornerTreatment=false;group.GapWithinGroup=0;group.GapForOtherGroups=0;group.MergeArcSegmentBodies=true;
            d.ClearSelection2(true);
            var member=d.FeatureManager.InsertStructuralWeldment4(profile,(int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter,true,new DispatchWrapper[]{new(group)}) as Feature??throw new Exception("Hoop weldment failed");member.Name=name;
            var newBodies=Arr(member.GetFaces()).Cast<Face2>().Select(face=>(Body2)face.GetBody()).GroupBy(b=>b.Name).Select(g=>g.First()).ToArray();
            for(int i=0;i<newBodies.Length;i++)newBodies[i].Name=name+"_"+(i+1);
            if(!d.EditRebuild3()||member.GetErrorCode()!=0)throw new Exception("Hoop rebuild error");
            records.Add(new {name,bodies=newBodies.Select(ReadBody).ToArray(),continuousSingleSolid=newBodies.Length==1});
            Save(d);
        }
        Mark(d,"RFR_Status","DEVELOPMENT — hoops only; no structural solve or compliance approval");Save(d);App.CloseDoc(d.GetTitle());d=Open(job.GetProperty("path").GetString()!);
        ChassisView(d);Save(d);var preview=Path.ChangeExtension(d.GetPathName(),"bmp");d.SaveBMP(preview,1600,1000);
        return new {hoops=records,reopened=Inspect(d),preview};
    }
}
