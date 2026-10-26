using System.Runtime.InteropServices;
using System.Text.Json;
using System.Reflection;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.cosworks;

// Narrow native engineering operations. No arbitrary COM invocation or global preference edits.
internal static partial class Program
{
    [DllImport("ole32.dll", CharSet=CharSet.Unicode)] static extern int CLSIDFromProgID(string name, out Guid id);
    [DllImport("oleaut32.dll", PreserveSig=false)] static extern void GetActiveObject(ref Guid id, IntPtr reserved, [MarshalAs(UnmanagedType.IUnknown)] out object value);
    static string Root = "";
    static ISldWorks App = null!;
    static readonly JsonSerializerOptions Json = new() { WriteIndented=true };
    static string Safe(string path) {
        var p=Path.GetFullPath(path);
        if (!new[]{"working","tests","exports"}.Any(d=>p.StartsWith(Path.Combine(Root,"cad",d)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)))
            throw new InvalidOperationException("CAD path must be within the project's working, tests, or exports directory.");
        return p;
    }
    static ModelDoc2 Open(string path) {
        path=Safe(path);
        var d=App.GetOpenDocumentByName(path) as ModelDoc2;
        int e=0,w=0;
        d ??= App.OpenDoc6(path,path.EndsWith(".sldasm",StringComparison.OrdinalIgnoreCase)?2:1,1,"",ref e,ref w) as ModelDoc2;
        if(d is null || e!=0) throw new Exception($"Open failed: {path}, error {e}, warning {w}");
        if(!string.Equals(Path.GetFullPath(d.GetPathName()),path,StringComparison.OrdinalIgnoreCase)) throw new Exception("Resolved document path mismatch");
        Activate(d); return d;
    }
    static void Activate(ModelDoc2 d) { int e=0; App.ActivateDoc3(d.GetTitle(),false,(int)swRebuildOnActivation_e.swDontRebuildActiveDoc,ref e); if(e!=0) throw new Exception($"Activate failed {e}"); }
    static ModelDoc2 New(bool assembly) => App.NewDocument(Path.Combine(Root,"cad/templates",assembly?"RFR-assembly-mm.asmdot":"RFR-part-mm.prtdot"),0,0,0) as ModelDoc2 ?? throw new Exception("Template create failed");
    static void SaveNew(ModelDoc2 d,string path) {
        path=Safe(path); if(File.Exists(path)) throw new Exception("Refusing to overwrite: "+path);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!); int e=0,w=0;
        if(!d.Extension.SaveAs(path,0,1,null,ref e,ref w)||e!=0)throw new Exception($"Save failed {e}, {w}: {path}");
    }
    static object[] Arr(object? v)=>v as object[]??[];
    static object ReadBody(Body2 b)=>new {name=b.Name,boundsM=b.GetBodyBox(),volumeM3=((double[])b.GetMassProperties(1))[3],faultCount=b.Check3?.Count??0,verticesM=Arr(b.GetVertices()).Cast<Vertex>().Select(v=>v.GetPoint()).ToArray(),cylinders=Arr(b.GetFaces()).Cast<Face2>().Where(f=>((Surface)f.GetSurface()).IsCylinder()).Select(f=>((Surface)f.GetSurface()).CylinderParams).ToArray()};
    static Body2[] Bodies(ModelDoc2 d)=>Arr(((IPartDoc)d).GetBodies2(0,false)).Cast<Body2>().ToArray();
    static object Inspect(ModelDoc2 d) {
        var fs=new List<object>();
        for(Feature? f=d.IFirstFeature();f!=null;f=f.IGetNextFeature()) {
            object? sketch=null;
            if(f.GetTypeName2() is "ProfileFeature" or "3DProfileFeature") {
                var s=(Sketch)f.GetSpecificFeature2();
                var segs=Arr(s.GetSketchSegments()).Cast<SketchSegment>().Select(seg=>{
                    object? detail=seg.GetType() switch {
                        0=>new {start=((SketchLine)seg).GetStartPoint2() is SketchPoint a?new[]{a.X,a.Y,a.Z}:null,end=((SketchLine)seg).GetEndPoint2() is SketchPoint b?new[]{b.X,b.Y,b.Z}:null},
                        1=>new {start=((SketchArc)seg).GetStartPoint2() is SketchPoint a?new[]{a.X,a.Y,a.Z}:null,end=((SketchArc)seg).GetEndPoint2() is SketchPoint b?new[]{b.X,b.Y,b.Z}:null,center=((SketchArc)seg).GetCenterPoint2() is SketchPoint c?new[]{c.X,c.Y,c.Z}:null,radius=((SketchArc)seg).GetRadius()},
                        _=>null
                    }; return new {kind=seg.GetType(),construction=seg.ConstructionGeometry,detail};
                }).ToArray();
                sketch=new {modelToSketch=s.ModelToSketchTransform.ArrayData,segments=segs};
            }
            fs.Add(new {name=f.Name,type=f.GetTypeName2(),error=f.GetErrorCode(),sketch});
        }
        object? cs=null,bs=null;
        if(d.GetType()==1)bs=Bodies(d).Select(ReadBody).ToArray();
        if(d.GetType()==2)cs=Arr(((IAssemblyDoc)d).GetComponents(false)).Cast<Component2>().Select(c=>new {name=c.Name2,path=c.GetPathName(),configuration=c.ReferencedConfiguration,transform=c.GetTotalTransform(true)?.ArrayData,boundsM=c.GetBox(false,false),suppressed=c.IsSuppressed(),bodyData=c.GetModelDoc2() is ModelDoc2 cd&&cd.GetType()==1?Bodies(cd).Select(ReadBody).ToArray():null}).ToArray();
        return new {path=d.GetPathName(),type=d.GetType(),configuration=d.ConfigurationManager.ActiveConfiguration.Name,dependencies=d.GetDependencies2(true,true,true),features=fs,components=cs,bodies=bs};
    }
    static object SnapshotPercy(JsonElement job) {
        var src=Open(job.GetProperty("path").GetString()!); if(src.GetType()!=2)throw new Exception("Expected imported assembly");
        var output=Safe(job.GetProperty("outputDir").GetString()!);
        if(Directory.Exists(output)&&Directory.EnumerateFileSystemEntries(output).Any()) throw new Exception("Output directory must be empty");
        Directory.CreateDirectory(output);
        var segments=new List<(string path,double volume)>(); var evidence=new List<object>();
        foreach(var c in Arr(((IAssemblyDoc)src).GetComponents(false)).Cast<Component2>()) {
            if(c.GetModelDoc2() is not ModelDoc2 pd||pd.GetType()!=1)continue;
            var bodies=Bodies(pd); if(bodies.Length!=1)throw new Exception($"Expected one solid in {c.Name2}");
            var b=(Body2)bodies[0].Copy(); var tf=c.GetTotalTransform(true); if(tf!=null&&!b.ApplyTransform(tf))throw new Exception("Copy transform failed");
            var name=Path.GetFileNameWithoutExtension(pd.GetTitle()).Replace(".stp","").Replace(" ","-");
            var target=Path.Combine(output,name+".SLDPRT");
            var n=New(false); var feat=((IPartDoc)n).CreateFeatureFromBody3(b,false,0) as Feature ?? throw new Exception("Independent body creation failed");
            feat.Name="PercyReferenceSegment"; n.EditRebuild3();
            var nb=Bodies(n); if(nb.Length!=1||nb[0].Check3?.Count>0)throw new Exception("Invalid cloned segment");
            double volume=((double[])b.GetMassProperties(1))[3], volume2=((double[])nb[0].GetMassProperties(1))[3];
            if(Math.Abs(volume-volume2)>1e-12)throw new Exception("Snapshot volume mismatch");
            SaveNew(n,target); segments.Add((target,volume)); evidence.Add(new {source=c.Name2,target,sourceTransform=tf?.ArrayData,body=ReadBody(nb[0])});
        }
        if(segments.Count!=11)throw new Exception($"Expected eleven segments; found {segments.Count}");
        var asm=New(true); string asmPath=Path.Combine(output,"RFR27-Percy-native.SLDASM");
        var mu=(MathUtility)App.GetMathUtility();
        foreach(var s in segments) {
            Activate(asm); var c=((AssemblyDoc)asm).AddComponent5(s.path,0,"",false,"",0,0,0) as Component2??throw new Exception("Insert segment failed");
            c.Transform2=mu.CreateTransform(new double[]{1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}) as MathTransform;
            asm.ClearSelection2(true); if(!c.Select4(false,null,false))throw new Exception("Select segment failed"); ((AssemblyDoc)asm).FixComponent();
        }
        asm.ClearSelection2(true); asm.EditRebuild3(); SaveNew(asm,asmPath);
        App.CloseDoc(asm.GetTitle()); foreach(var s in segments)App.CloseDoc(Path.GetFileName(s.path));
        var reopened=Open(asmPath); var after=Inspect(reopened);
        reopened.ShowNamedView2("*Isometric",7); reopened.ViewZoomtofit2(); string preview=Path.Combine(output,"Percy-standing.bmp"); if(!reopened.SaveBMP(preview,1600,1000))throw new Exception("Preview failed");
        return new {assembly=asmPath,segments=evidence,reopened=after,preview,pose="Imported standing reference, not fitted to chassis"};
    }
    static object SimulationProbe(JsonElement job) {
        var doc=Open(job.GetProperty("path").GetString()!);
        string dll=Path.Combine(App.GetExecutablePath(),"Simulation","cosworks.dll");
        int? unload=job.TryGetProperty("reload",out var reload)&&reload.GetBoolean()?App.UnloadAddIn(dll):null;
        object? before=null;string? beforeError=null;int? load=null;
        try{before=App.GetAddInObject("SldWorks.Simulation");}catch(Exception e){beforeError=e.Message;}
        if(before==null&&job.TryGetProperty("load",out var l)&&l.GetBoolean())load=App.LoadAddIn(Path.Combine(App.GetExecutablePath(),"Simulation","cosworks.dll"));
        var probes=new List<object>(); ICwAddincallback? add=null;
        foreach(var prog in new[]{"SldWorks.Simulation","SldWorks.Simulation."+(int.Parse(App.RevisionNumber().Split('.')[0])-15),"SldWorks.CosmosWorks"}) {
            try {var raw=App.GetAddInObject(prog); probes.Add(new {prog,available=raw!=null}); add??=raw as ICwAddincallback;} catch(Exception e){probes.Add(new {prog,error=e.Message});}
        }
        var cosmos=add?.CosmosWorks; var ad=cosmos?.ActiveDoc; var manager=ad?.StudyManager;
        return new {nativeDll=dll,unloadResult=unload,beforeError,loadedBefore=before!=null,loadResult=load,loadMeaning=load==null?null:((swLoadAddinError_e)load).ToString(),probes,callbackAvailable=add!=null,cosmosAvailable=cosmos!=null,activeStudyDocumentAvailable=ad!=null,studyCount=manager?.StudyCount,licenseVerifiedBySolve=false,document=doc.GetPathName()};
    }
    static object FinishEngineeringSession(JsonElement job) {
        var saved=new List<string>();
        string beamPath=Safe(Path.Combine(Root,"cad/tests/simulation-acceptance-001/beam-tube.SLDPRT"));
        if(App.GetOpenDocumentByName(beamPath) is ModelDoc2 beam){
            string before=Path.Combine(Path.GetDirectoryName(beamPath)!,"beam-tube-before-experimental-studies.SLDPRT");
            if(!File.Exists(before))File.Copy(beamPath,before);
            Save(beam);saved.Add(beamPath);App.CloseDoc(beam.GetTitle());
        }
        foreach(var d in Arr(App.GetDocuments()).Cast<ModelDoc2>()){
            if(d.GetPathName()==""&&d.GetTitle()=="Part45"&&d.GetType()==1&&Bodies(d).Length==0){
                string failed=Safe(Path.Combine(Root,"cad/tests/simulation-acceptance-001/failed-empty-beam-attempt.SLDPRT"));
                SaveNew(d,failed);saved.Add(failed);App.CloseDoc(d.GetTitle());
            }
        }
        var active=Open(job.GetProperty("path").GetString()!);
        return new {saved,activeDocument=new {path=active.GetPathName(),title=active.GetTitle(),configuration=active.ConfigurationManager.ActiveConfiguration.Name},
            openDocuments=Arr(App.GetDocuments()).Cast<ModelDoc2>().Select(d=>new {path=d.GetPathName(),title=d.GetTitle(),dirty=d.GetSaveFlag()}).ToArray()};
    }
    [STAThread] static int Main(string[] args) {
        try {
            Root=Path.GetFullPath(args[0]); var job=JsonDocument.Parse(File.ReadAllText(args[1])).RootElement; string command=job.GetProperty("command").GetString()!;
            if(command=="api-signatures") { var term=job.GetProperty("term").GetString()!; var output=new[]{typeof(ISldWorks).Assembly,typeof(ICosmosWorks).Assembly,typeof(swLoadAddinError_e).Assembly}.SelectMany(a=>a.GetTypes()).Where(t=>t.Name.Contains(term,StringComparison.OrdinalIgnoreCase)).Select(t=>new {type=t.FullName,members=t.GetMembers().Select(m=>m.ToString()).ToArray()}); Console.WriteLine(JsonSerializer.Serialize(output,Json)); return 0; }
            // Reject the known interactive experiments before connecting to CAD.
            if(command=="beam-macro-binding")throw new NotSupportedException("In-process joint picking also enters interactive manipulation. Use fixture context, never direct picking.");
            if(command=="joint-selection-test")JointSelectionTest(job);
            CLSIDFromProgID("SldWorks.Application",out var id); GetActiveObject(ref id,IntPtr.Zero,out var app); App=(ISldWorks)app;
            object result=command switch {"joint-fit-audit"=>JointFitAudit(job),"packaging-audit"=>PackagingAudit(job),"chassis-simulation"=>ChassisSimulation(job),"trim-connecting-frame"=>TrimConnectingFrame(job),"build-connecting-frame"=>BuildConnectingFrame(job),"frame-acceptance"=>FrameAcceptance(job),"beam-context-binding"=>BeamContextBinding(job),"repair-main-hoop"=>RepairMainHoop(job),"chassis-audit"=>ChassisAudit(job),"beam-face-binding"=>BeamFaceBinding(job),"upright-view"=>UprightView(job),"snapshot"=>Snapshot(job),"joint-probe"=>JointProbe(job),"joint-diagnostics"=>JointDiagnostics(job),"export-study"=>ExportStudy(job),"joint-pick-test"=>JointPickTest(job),"study-settings"=>StudySettings(job),"file-info"=>FileInfo(job),"drawing-notes"=>DrawingNotes(job),"presentation-views"=>PresentationViews(job),"show-for-screenshot"=>ShowForScreenshot(job),"close-doc"=>CloseProjectDoc(job),"export-points"=>ExportComponentPoints(job),"build-trim-frame"=>BuildTrimFrame(job),"recover-part"=>RecoverPart(job),"vr3-package"=>Vr3Package(job), "vr3-drawing" => Vr3Drawing(job), "make-profiles" => MakeProfiles(job),"finish-session"=>FinishEngineeringSession(job),"selection-diagnostics"=>SelectionDiagnostics(job),"joint-selection-test"=>JointSelectionTest(job),"simulation-beam-solve"=>SimulationBeamSolve(job),"simulation-beam-test"=>SimulationBeamTest(job),"simulation-solid-test"=>SimulationSolidTest(job),"session-inventory"=>Arr(App.GetDocuments()).Cast<ModelDoc2>().Select(d=>new {title=d.GetTitle(),path=d.GetPathName(),dirty=d.GetSaveFlag(),type=d.GetType()}).ToArray(),"inspect"=>Inspect(Open(job.GetProperty("path").GetString()!)),"snapshot-percy"=>SnapshotPercy(job),"simulation-probe"=>SimulationProbe(job),"pose-assembly"=>PoseAssembly(job),"build-hoops"=>BuildHoops(job),_=>throw new Exception("Unknown typed command")};
            Console.WriteLine(JsonSerializer.Serialize(new {ok=true,completedUtc=DateTime.UtcNow,result},Json)); return 0;
        }catch(Exception e){Console.WriteLine(JsonSerializer.Serialize(new {ok=false,error=e.ToString()},Json));return 1;}
    }
}













