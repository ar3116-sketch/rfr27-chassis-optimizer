using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    // Bring a cad/tests document to the front for a person's screenshot: active document, SOLIDWORKS
    // window maximized, chassis isometric zoomed to fit. Never saves anything.
    static object ShowForScreenshot(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        if(!path.Contains(Path.Combine("cad","tests")))throw new Exception("Screenshot setup uses a cad/tests copy");
        var d=Open(path);Activate(d);
        App.FrameState=(int)swWindowState_e.swWindowMaximized;
        ChassisView(d);
        return new {path,active=((ModelDoc2)App.ActiveDoc).GetTitle(),frameState=App.FrameState,saved=false};
    }
}
