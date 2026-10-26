using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    // Presentation images of a frame for people to look at: tubes coloured per body (job "colors":
    // {bodyName:[r,g,b]}), shaded with edges, reference geometry hidden; overall iso/side/top views and
    // iso close-ups (job "closeups": [{name, center:[x,y,z] m, half m}]). Works on a cad/tests copy only,
    // never saves it, and leaves it open in the isometric view so a person can take screenshots.
    static object PresentationViews(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        if(!path.Contains(Path.Combine("cad","tests")))throw new Exception("Presentation views use a cad/tests copy");
        string dir=Path.GetFullPath(job.GetProperty("outputDir").GetString()!);
        if(!dir.StartsWith(Path.GetFullPath(Root).TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))
            throw new Exception("Output directory must be inside the project");
        Directory.CreateDirectory(dir);
        var d=Open(path);Activate(d);
        int colored=0;
        if(job.TryGetProperty("colors",out var colors))
            foreach(var b in Bodies(d))if(colors.TryGetProperty(b.Name,out var c)){var rgb=Numbers(c);b.MaterialPropertyValues2=new double[]{rgb[0],rgb[1],rgb[2],.55,.9,.4,.35,0,0};colored++;}
        d.Extension.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swViewDisplayHideAllTypes,(int)swUserPreferenceOption_e.swDetailingNoOptionSpecified,true);
        var v=d.ActiveView as ModelView??throw new Exception("No view");
        v.DisplayMode=(int)swViewDisplayMode_e.swViewDisplayMode_ShadedWithEdges;
        var mu=(MathUtility)App.GetMathUtility();
        double[] top={1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0};
        var shots=new List<string>();
        void Shot(string name,double[] m,double[]? center,double half,int w,int h) {
            v.Orientation3=(MathTransform)mu.CreateTransform(m);
            if(center==null)d.ViewZoomtofit2();
            else d.ViewZoomTo2(center[0]-half,center[1]-half,center[2]-half,center[0]+half,center[1]+half,center[2]+half);
            d.GraphicsRedraw2();
            string f=Path.Combine(dir,name+".bmp");if(!d.SaveBMP(f,w,h))throw new Exception("Image failed: "+name);shots.Add(f);
        }
        var isoSize=job.TryGetProperty("isoSize",out var iz)?Numbers(iz):new double[]{2400,1500};
        Shot("iso",ChassisViewMatrix(),null,0,(int)isoSize[0],(int)isoSize[1]);
        Shot("side",ChassisViewMatrix(true),null,0,2400,1200);
        Shot("top",top,null,0,2400,1200);
        if(job.TryGetProperty("closeups",out var cu))
            foreach(var c in cu.EnumerateArray())Shot(c.GetProperty("name").GetString()!,ChassisViewMatrix(),Numbers(c.GetProperty("center")),c.GetProperty("half").GetDouble(),1800,1350);
        v.Orientation3=(MathTransform)mu.CreateTransform(ChassisViewMatrix());d.ViewZoomtofit2();d.GraphicsRedraw2();
        return new {path,colored,shots,leftOpen=d.GetTitle(),saved=false};
    }
}
