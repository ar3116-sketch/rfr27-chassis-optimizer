using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    // Diagnostic only: which beam joint does a coordinate pick return in a saved study, for a
    // set of views, zooms and pick-point offsets? Enters the fixture-selection context, never
    // adds a fixture or load, cancels the context and closes the test copy without saving.
    static object JointPickTest(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        if(!path.Contains(Path.Combine("cad","tests")))throw new Exception("Pick tests run on a cad/tests copy only");
        var d=Open(path);Activate(d);
        var cw=RequireSimulation();var mgr=cw.ActiveDoc.StudyManager;
        string name=job.GetProperty("study").GetString()!;int idx=-1;
        for(int i=0;i<mgr.StudyCount;i++)if(mgr.GetStudy(i).Name==name)idx=i;
        if(idx<0)throw new Exception("Study not found");mgr.ActiveStudy=idx;
        var callback=(ICwAddincallback)App.GetAddInObject("SldWorks.Simulation."+(int.Parse(App.RevisionNumber().Split('.')[0])-15));
        var sm=(SelectionMgr)d.SelectionManager;var results=new List<object>();
        object Pick(double[] q){d.ClearSelection2(true);bool ok=d.Extension.SelectByID2("","SIMULATION",q[0],q[1],q[2],false,0,null,0);
            object? o=ok?sm.GetSelectedObject6(1,-1):null;
            return new {ok,type=ok?sm.GetSelectedObjectType3(1,-1):-1,name=(o as IManipulator)?.Name};}
        callback.DefineRestraints();
        try {
            // Each case: point, view ("current" = the saved view, untouched), zoom half-size h (m)
            // around the point, and pick offsets (m) added to the point.
            foreach(var c in job.TryGetProperty("cases",out var cs)?cs.EnumerateArray().ToArray():Array.Empty<JsonElement>()) {
                var p=Numbers(c.GetProperty("point"));string view=c.GetProperty("view").GetString()!;
                double h=c.TryGetProperty("h",out var hh)?hh.GetDouble():0;
                if(view!="current"){d.ShowNamedView2(view,-1);if(h>0)d.ViewZoomTo2(p[0]-h,p[1]-h,p[2]-h,p[0]+h,p[1]+h,p[2]+h);}
                var orientation=((ModelView)d.ActiveView).Orientation3.ArrayData as double[];
                results.Add(new {label=c.TryGetProperty("label",out var l0)?l0.GetString():null,view,h,orientation});
                foreach(var off in c.GetProperty("offsets").EnumerateArray().Select(Numbers)) {
                    var q=new[]{p[0]+off[0],p[1]+off[1],p[2]+off[2]};
                    results.Add(new {label=c.TryGetProperty("label",out var l)?l.GetString():null,view,h,offsetMm=off.Select(x=>x*1000).ToArray(),pick=Pick(q)});
                }
            }
            // Localise the joints near each listed point, as the binder's pass 3 does.
            if(job.TryGetProperty("localize",out var lz)) {
                var views=AxisViews(d);double reach=job.TryGetProperty("reachM",out var rm)?rm.GetDouble():.01;
                foreach(var p in lz.EnumerateArray().Select(Numbers)) {
                    var timer=System.Diagnostics.Stopwatch.StartNew();var found=LocalizeJoints(d,views,p,reach,null,exhaustive:true).found;
                    results.Add(new {localize=p,views,seconds=timer.Elapsed.TotalSeconds,candidates=found.Select(c=>new {c.name,offsetMm=c.offsetM.Select(x=>x*1000).ToArray(),distanceMm=c.distanceM*1000,c.planHits,c.depthHits}).ToArray()});
                }
            }
        } finally {
            d.ShowNamedView2("*Isometric",-1);
            if(!App.RunCommand(3043,""))Stage("Could not close native fixture-selection context");
            d.ClearSelection2(true);
        }
        bool dirty=d.GetSaveFlag();App.CloseDoc(d.GetTitle());
        return new {path,study=name,dirtyBeforeCloseWithoutSave=dirty,results};
    }
}

internal static partial class Program
{
    // Diagnostic only: read an existing study's beam-joint group and mesh settings (getters only;
    // no begin/end edit, mesh or solve). A document this call opened is closed without saving;
    // one that was already open (e.g. a hand-run study left open in the UI) is left open.
    static object StudySettings(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        if(!path.Contains(Path.Combine("cad","tests")))throw new Exception("Study settings are read from a cad/tests file only");
        bool wasOpen=App.GetOpenDocumentByName(path) is ModelDoc2;
        var d=Open(path);Activate(d);
        var mgr=RequireSimulation().ActiveDoc.StudyManager;var studies=new List<object>();
        for(int i=0;i<mgr.StudyCount;i++) {
            var s=mgr.GetStudy(i);var bm=s.BeamManager;var j=bm.GetJointGroup(out int je);
            var beams=new List<string>();for(int k=0;k<bm.BeamCount;k++)beams.Add(bm.GetBeamBodyAt(k,out int _)?.BeamBodyName??"");
            studies.Add(new {name=s.Name,beamCount=bm.BeamCount,beams,jointGroupError=je,
                joints=j==null?null:new {j.IncludeAllSelectedBeam,j.IncludeUserSelectedBeam,j.IncludeTreatAsJointForClearanceLessThan,j.IncludeKeepModifiedJointOnUpdate,j.IncludeDisplayNeutralAxis,j.PinBallRadius,j.PinBallRadiusUnit},
                mesh=new {s.Mesh.MeshType,nodeCount=s.Mesh.NodeCount,elementCount=s.Mesh.ElementCount}});
        }
        bool dirty=d.GetSaveFlag();if(!wasOpen)App.CloseDoc(d.GetTitle());
        return new {path,cadSha256=FileSha(path),wasOpen,dirty,closed=!wasOpen,studies};
    }
}

internal static partial class Program
{
    // Read-only file facts for handing files to someone else: the SOLIDWORKS versions a file was
    // saved in (VersionHistory) and the files it references (GetDocumentDependencies2). Neither
    // call opens the document.
    static object FileInfo(JsonElement job) => job.GetProperty("paths").EnumerateArray().Select(x=>x.GetString()!).Select(p=>new {
        path=p,versionHistory=Arr(App.VersionHistory(p)).Select(v=>v?.ToString()).ToArray(),
        dependencies=Arr(App.GetDocumentDependencies2(p,true,true,false)).Select(v=>v?.ToString()).ToArray()}).ToArray();
}

internal static partial class Program
{
    // Read-only: the notes in each sheet's format (title block) with their property links, and the
    // drawing's custom properties. Closes the drawing without saving.
    static object DrawingNotes(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        bool wasOpen=App.GetOpenDocumentByName(path) is ModelDoc2;int e=0,w=0;
        var d=App.GetOpenDocumentByName(path) as ModelDoc2??App.OpenDoc6(path,(int)SolidWorks.Interop.swconst.swDocumentTypes_e.swDocDRAWING,1,"",ref e,ref w) as ModelDoc2??throw new Exception($"Open drawing failed {e},{w}");
        Activate(d);var drw=(DrawingDoc)d;var sheets=new List<object>();
        try {
            foreach(var name in Arr(drw.GetSheetNames()).Select(x=>x!.ToString()!)) {
                drw.ActivateSheet(name);drw.EditTemplate();var notes=new List<object>();
                for(var v=(View?)drw.GetFirstView();v!=null;v=(View?)v.GetNextView()) {
                    foreach(var n in Arr(v.GetNotes()).OfType<Note>())notes.Add(new {view=v.GetName2(),text=n.GetText(),linked=n.PropertyLinkedText,height=n.GetHeight(),pos=n.GetTextPoint2()});
                    break; // the sheet itself carries the format notes
                }
                drw.EditSheet();sheets.Add(new {name,notes});
            }
            var cpm=d.Extension.CustomPropertyManager[""];var props=Arr(cpm.GetNames()).Select(x=>x!.ToString()!).ToDictionary(k=>k,k=>{cpm.Get6(k,false,out string raw,out string val,out bool _,out bool _);return new{raw,val};});
            return new {path,sheets,props};
        } finally {if(!wasOpen)App.CloseDoc(d.GetTitle());}
    }
}
