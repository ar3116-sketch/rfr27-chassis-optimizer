using System.Text.Json;
using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    // Chassis coordinates are X forward, Y left, Z up. Only the camera changes.
    static double[] ChassisViewMatrix(bool side=false) {
        if(side)return new double[]{1,0,0,0,0,-1,0,1,0,0,0,0,1,0,0,0};
        double a=1/Math.Sqrt(2),b=1/Math.Sqrt(6),c=1/Math.Sqrt(3);
        return new double[]{a,-b,c,a,b,-c,0,2*b,c,0,0,0,1,0,0,0};
    }
    static void ChassisView(ModelDoc2 d,bool side=false) {
        var v=(ModelView)d.ActiveView;
        v.Orientation3=(MathTransform)((MathUtility)App.GetMathUtility()).CreateTransform(ChassisViewMatrix(side));
        d.ViewZoomtofit2();if(side)v.Scale2*=.7;d.GraphicsRedraw2();
    }
    // Visual checkpoint: isometric and side BMPs, camera restored afterwards. Never saves the document.
    static object SaveChassisImages(ModelDoc2 d,string dir,string stem) {
        dir=Path.GetFullPath(dir);
        if(!dir.StartsWith(Path.GetFullPath(Root).TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))
            throw new Exception("Snapshot directory must be inside the project");
        Directory.CreateDirectory(dir);
        if(d.ActiveView==null){int e=0,w=0;App.OpenDoc6(d.GetPathName(),d.GetType(),1,"",ref e,ref w);}
        Activate(d);
        var v=d.ActiveView as ModelView??throw new Exception("No view available for snapshot");
        var original=(double[])v.Orientation3.ArrayData;
        string iso=Path.Combine(dir,stem+"-iso.bmp"),side=Path.Combine(dir,stem+"-side.bmp");
        ChassisView(d);if(!d.SaveBMP(iso,1600,1000))throw new Exception("Isometric snapshot failed");
        ChassisView(d,true);if(!d.SaveBMP(side,1600,1000))throw new Exception("Side snapshot failed");
        v.Orientation3=(MathTransform)((MathUtility)App.GetMathUtility()).CreateTransform(original);d.GraphicsRedraw2();
        return new {iso,side};
    }
    static object Snapshot(JsonElement job) {
        string dir=job.GetProperty("outputDir").GetString()!;var shots=new List<object>();
        foreach(var p in job.GetProperty("paths").EnumerateArray()) {
            var d=Open(p.GetString()!);bool dirty=d.GetSaveFlag();
            var images=SaveChassisImages(d,dir,Path.GetFileNameWithoutExtension(d.GetPathName()));
            shots.Add(new {path=d.GetPathName(),images,dirtyBefore=dirty,dirtyAfter=d.GetSaveFlag()});
        }
        if(job.TryGetProperty("restore",out var restore))Activate(Open(restore.GetString()!));
        return new {shots,saved=false};
    }
    static object UprightView(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);
        string State()=>JsonSerializer.Serialize(d.GetType()==2
            ?Arr(((AssemblyDoc)d).GetComponents(false)).Cast<Component2>().Select(c=>new {path=c.GetPathName(),tf=c.GetTotalTransform(true)?.ArrayData}).ToArray()
            :(object)Bodies(d).Select(ReadBody).ToArray(),Json);
        string before=State();
        var original=((ModelView)d.ActiveView).Orientation3.ArrayData;
        ChassisView(d,true);d.NameView("RFR Side - Z up");
        string side=Path.ChangeExtension(d.GetPathName(),"z-up-side.bmp");
        if(!d.SaveBMP(side,1600,1000))throw new Exception("Side preview failed");
        ChassisView(d);d.NameView("RFR Isometric - Z up");
        var expected=ChassisViewMatrix();
        var actual=(double[])((ModelView)d.ActiveView).Orientation3.ArrayData;
        if(actual.Take(9).Zip(expected.Take(9),(a,b)=>Math.Abs(a-b)).Max()>1e-9)throw new Exception("View orientation readback mismatch");
        if(before!=State())throw new Exception("Geometry or component placement changed during view correction");
        Save(d);
        string preview=Path.ChangeExtension(d.GetPathName(),"z-up.bmp");
        if(!d.SaveBMP(preview,1600,1000))throw new Exception("Isometric preview failed");
        return new {path=d.GetPathName(),configuration=d.ConfigurationManager.ActiveConfiguration.Name,originalOrientation=original,
            orientation=actual,geometryAndPlacementUnchanged=true,sidePreview=side,preview,saved=true};
    }
}
