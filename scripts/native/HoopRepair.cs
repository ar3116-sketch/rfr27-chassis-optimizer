using System.Text.Json;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    static object RepairMainHoop(JsonElement job) {
        string source=Safe(job.GetProperty("source").GetString()!),output=Safe(job.GetProperty("output").GetString()!);
        if(File.Exists(output))throw new Exception("Fresh working candidate required");
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(source,output);
        var d=Open(output);var old=Find(d,"MainHoop")??throw new Exception("Original working main hoop feature missing");
        if(!old.SetSuppression2((int)swFeatureSuppressionAction_e.swSuppressFeature,(int)swInConfigurationOpts_e.swAllConfiguration,null))throw new Exception("Could not suppress old working hoop");
        d.EditRebuild3();Stage("Old working hoop suppressed; original path retained");
        d.ClearSelection2(true);var sm=d.SketchManager;sm.Insert3DSketch(true);var sketch=(Sketch)sm.ActiveSketch;var created=new List<SketchSegment>();
        sm.AddToDB=true;
        try {
            foreach(var s in recipe.GetProperty("segments").EnumerateArray()) {
                var a=Numbers(s.GetProperty("start"));var b=Numbers(s.GetProperty("end"));SketchSegment? seg;
                if(s.GetProperty("kind").GetString()=="line")seg=sm.CreateLine(a[0],a[1],a[2],b[0],b[1],b[2]);
                else {var c=Numbers(s.GetProperty("mid"));seg=sm.Create3PointArc(a[0],a[1],a[2],b[0],b[1],b[2],c[0],c[1],c[2]);}
                created.Add(seg??throw new Exception("Native path segment failed"));
            }
        } finally {sm.AddToDB=false;sm.Insert3DSketch(true);}
        var sf=d.IFeatureByPositionReverse(0);if(sf.GetTypeName2()!="3DProfileFeature")throw new Exception("New hoop sketch not found");sf.Name="MainHoop_TangentPath";
        var group=d.FeatureManager.CreateStructuralMemberGroup();group.Segments=created.ToArray();group.ApplyCornerTreatment=false;
        group.MergeArcSegmentBodies=true;group.GapWithinGroup=0;group.GapForOtherGroups=0;
        d.ClearSelection2(true);Stage("Building tangent continuous hoop");
        var f=d.FeatureManager.InsertStructuralWeldment4(Path.Combine(Root,"profiles",recipe.GetProperty("profile").GetString()!),
            (int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter,true,new DispatchWrapper[]{new(group)})??throw new Exception("Native repaired hoop failed");
        f.Name="MainHoop_Repaired";
        var bodies=Arr(f.GetFaces()).Cast<Face2>().Select(x=>(Body2)x.GetBody()).GroupBy(x=>x.Name).Select(x=>x.First()).ToArray();
        for(int i=0;i<bodies.Length;i++)bodies[i].Name="MainHoop_"+(i+1);
        bool rebuilt=d.EditRebuild3();Save(d);
        if(!rebuilt||f.GetErrorCode()!=0||bodies.Length!=1||bodies.Any(b=>b.Check3?.Count!=0))throw new Exception($"Repaired hoop rejected: rebuild={rebuilt}, featureError={f.GetErrorCode()}, bodies={bodies.Length}");
        Mark(d,"RFR_MainHoopRecipe",job.GetProperty("recipe").GetString()!);
        Mark(d,"RFR_Status","Tangent continuous main hoop; complete frame and overall compliance still in development");
        ChassisView(d);Save(d);App.CloseDoc(d.GetTitle());d=Open(output);
        var saved=Bodies(d).Where(b=>b.Name.StartsWith("MainHoop_")).ToArray();
        if(saved.Length!=1||saved[0].Check3?.Count!=0)throw new Exception("Reopened hoop continuity failed");
        var preview=Path.ChangeExtension(output,"bmp");d.SaveBMP(preview,1600,1000);
        return new {path=output,configuration=d.ConfigurationManager.ActiveConfiguration.Name,mainHoopBodies=saved.Select(ReadBody).ToArray(),
            rebuilt=true,saveReopenPassed=true,sourceSketchPreserved=true,recipe=job.GetProperty("recipe").GetString(),preview};
    }
}
