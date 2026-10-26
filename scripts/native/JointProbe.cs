using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // Diagnostic only: can beam joints be picked by coordinate in an existing study?
    static object JointProbe(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);Activate(d);
        var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;
        string name=job.GetProperty("study").GetString()!;int idx=-1;
        for(int i=0;i<mgr.StudyCount;i++)if(mgr.GetStudy(i).Name==name)idx=i;
        if(idx<0)throw new Exception("Study not found");mgr.ActiveStudy=idx;
        var callback=(ICwAddincallback)App.GetAddInObject("SldWorks.Simulation."+(int.Parse(App.RevisionNumber().Split('.')[0])-15));
        var sm=(SelectionMgr)d.SelectionManager;var results=new List<object>();
        bool zoomed=false;
        foreach(var pass in new[]{"metres","inches","mm"}) {
            double f=pass=="inches"?1/0.0254:pass=="mm"?1000:1;
            callback.DefineRestraints();
            try {
                foreach(var p in job.GetProperty("points").EnumerateArray().Select(Numbers)) {
                    d.ClearSelection2(true);bool ok=d.Extension.SelectByID2("","SIMULATION",p[0]*f,p[1]*f,p[2]*f,false,0,null,0);
                    object? o=ok?sm.GetSelectedObject6(1,-1):null;
                    results.Add(new {pass,point=p,ok,type=ok?sm.GetSelectedObjectType3(1,-1):-1,name=(o as IManipulator)?.Name});
                }
            } finally {App.RunCommand(3043,"");d.ClearSelection2(true);}
        }
        var v=(ModelView)d.ActiveView;
        return new {results,zoomed,viewScale=v?.Scale2,graphicsUpdate=v?.EnableGraphicsUpdate,frameState=App.FrameState,visible=App.Visible};
    }
}

internal static partial class Program
{
    // Close a project working/test document without saving (never anything outside cad/).
    static object CloseProjectDoc(JsonElement job) {
        if(job.TryGetProperty("unsavedDrawingTitle",out var t)) {
            // Only never-saved drawings (no path) that this tool creates for VR3 packages.
            var dd=Arr(App.GetDocuments()).Cast<ModelDoc2>().FirstOrDefault(x=>x.GetTitle()==t.GetString()&&x.GetType()==3&&string.IsNullOrEmpty(x.GetPathName()));
            if(dd==null)return new {title=t.GetString(),wasOpen=false};
            App.CloseDoc(dd.GetTitle());return new {title=t.GetString(),wasOpen=true};
        }
        string path=Safe(job.GetProperty("path").GetString()!);
        var d=App.GetOpenDocumentByName(path) as ModelDoc2;if(d==null)return new {path,wasOpen=false};
        App.CloseDoc(d.GetTitle());return new {path,wasOpen=true,stillOpen=App.GetOpenDocumentByName(path)!=null};
    }
}
