using System.Text.Json;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    static string FileSha(string path) {
        using var stream=new FileStream(path,FileMode.Open,FileAccess.Read,FileShare.ReadWrite);
        return Convert.ToHexString(SHA256.HashData(stream)).ToLowerInvariant();
    }
    static double Volume(Body2 b)=>((double[])b.GetMassProperties(1))[3];
    static object ProtectedGeometry(ModelDoc2 d,string[] sketches,string[] planes,string[] refs) {
        static double[] Point(SketchPoint p)=>new[]{p.X,p.Y,p.Z};
        var ss=sketches.Select(n=>{
            var f=Find(d,n)??throw new Exception("Missing protected sketch "+n);
            var s=(Sketch)f.GetSpecificFeature2();
            var segments=Arr(s.GetSketchSegments()).Cast<SketchSegment>().Select(seg=>{
                object detail=seg.GetType() switch {
                    0=>new {start=Point((SketchPoint)((SketchLine)seg).GetStartPoint2()),end=Point((SketchPoint)((SketchLine)seg).GetEndPoint2())},
                    1=>new {start=Point((SketchPoint)((SketchArc)seg).GetStartPoint2()),end=Point((SketchPoint)((SketchArc)seg).GetEndPoint2()),center=Point((SketchPoint)((SketchArc)seg).GetCenterPoint2()),radius=((SketchArc)seg).GetRadius()},
                    _=>throw new Exception("Unsupported protected sketch segment; cannot prove reference preservation")
                };return new {kind=seg.GetType(),construction=seg.ConstructionGeometry,detail};
            }).ToArray();
            return new {name=n,modelToSketch=s.ModelToSketchTransform.ArrayData,segments};
        }).ToArray();
        var ps=planes.Select(n=>new {name=n,transform=((RefPlane)(Find(d,n)??throw new Exception("Missing plane "+n)).GetSpecificFeature2()).Transform.ArrayData}).ToArray();
        var bs=refs.Select(n=>{
            var b=Bodies(d).Single(b=>b.Name==n);return new {name=n,bounds=b.GetBodyBox(),volume=Volume(b)};
        }).ToArray();
        return new {sketches=ss,planes=ps,referenceBodies=bs};
    }
    static bool SameGeometry(JsonElement a,JsonElement b) {
        if(a.ValueKind!=b.ValueKind)return false;
        if(a.ValueKind==JsonValueKind.Number)return Math.Abs(a.GetDouble()-b.GetDouble())<=1e-9;
        if(a.ValueKind==JsonValueKind.Array)return a.GetArrayLength()==b.GetArrayLength()&&a.EnumerateArray().Zip(b.EnumerateArray(),SameGeometry).All(v=>v);
        if(a.ValueKind==JsonValueKind.Object)return a.EnumerateObject().Count()==b.EnumerateObject().Count()&&a.EnumerateObject().All(p=>b.TryGetProperty(p.Name,out var v)&&SameGeometry(p.Value,v));
        return a.GetRawText()==b.GetRawText();
    }
    sealed record PairCheck(string first,string second,bool broadPhaseDisjoint,bool resolved,int booleanError,double? overlapM3);
    static PairCheck Intersection(Body2 a,Body2 b,string an,string bn) {
        var aa=(double[])a.GetBodyBox();var bb=(double[])b.GetBodyBox();
        // GetBodyBox is conservative; use it only to exclude distant pairs.
        if(Enumerable.Range(0,3).Any(i=>aa[i+3]<bb[i]-1e-8||bb[i+3]<aa[i]-1e-8))return new(an,bn,true,true,0,0);
        var ac=(Body2)a.Copy();var bc=(Body2)b.Copy();
        try {
            var common=Arr(ac.Operations2((int)swBodyOperationType_e.SWBODYINTERSECT,bc,out int e)).Cast<Body2>().ToArray();
            double vol=common.Sum(Volume);foreach(var v in common)Marshal.ReleaseComObject(v);
            bool resolved=e==0||e==(int)swBodyOperationError_e.swBodyOperationDisjointBodies||e==(int)swBodyOperationError_e.swBodyOperationNoIntersect||e==(int)swBodyOperationError_e.swBodyOperationEmptyBody;
            return new(an,bn,false,resolved,e,resolved?vol:null);
        } finally {Marshal.ReleaseComObject(ac);Marshal.ReleaseComObject(bc);}
    }
    static object ChassisAudit(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        string baseline=Safe(job.GetProperty("baseline").GetString()!);
        string percy=Safe(job.GetProperty("percy").GetString()!);
        string restore=Safe(job.GetProperty("restore").GetString()!);
        string snapshot=Safe(job.GetProperty("snapshot").GetString()!);
        if(!snapshot.Contains(Path.Combine("cad","tests"))||File.Exists(snapshot))throw new Exception("Fresh audit snapshot required");
        // Never score a stale disk version of a dirty live candidate.
        if(App.GetOpenDocumentByName(path) is ModelDoc2 current&&current.GetSaveFlag())throw new Exception("Candidate has unsaved changes; save an explicit working checkpoint before evaluation");
        Directory.CreateDirectory(Path.GetDirectoryName(snapshot)!);File.Copy(path,snapshot);
        var ledger=JsonDocument.Parse(File.ReadAllText(Path.Combine(Root,"design/constraint-ledger.json"))).RootElement;
        string[] Strings(string key)=>ledger.GetProperty(key).EnumerateArray().Select(v=>v.GetString()!).ToArray();
        var referenceNames=Strings("referenceOnlyBodies");var sketches=Strings("protectedSketches");var planes=Strings("protectedPlanes");
        var baseDoc=Open(baseline);var protectedBase=JsonSerializer.SerializeToElement(ProtectedGeometry(baseDoc,sketches,planes,referenceNames),Json);
        var d=Open(snapshot);
        try {
            string config=job.GetProperty("configuration").GetString()!;
            if(d.ConfigurationManager.ActiveConfiguration.Name!=config)d.ShowConfiguration2(config);
            if(d.ConfigurationManager.ActiveConfiguration.Name!=config)throw new Exception("Configuration unavailable: "+config);
            Stage("Rebuilding owned audit snapshot");bool rebuilt=d.EditRebuild3();
            var all=Bodies(d);var refs=referenceNames.Select(n=>all.Single(b=>b.Name==n)).ToArray();
            var structure=all.Where(b=>!referenceNames.Contains(b.Name)).ToArray();
            if(structure.Length==0)throw new Exception("No structural bodies");
            object? mainHoopAudit=job.TryGetProperty("mainHoopRecipe",out var recipe)?AuditMainHoop(d,recipe.GetString()!):null;
            var protectedNow=JsonSerializer.SerializeToElement(ProtectedGeometry(d,sketches,planes,referenceNames),Json);
            var errors=new List<object>();
            for(Feature? f=d.IFirstFeature();f!=null;f=f.IGetNextFeature())if(f.GetErrorCode()!=0)errors.Add(new {name=f.Name,error=f.GetErrorCode()});
            var keepout=new List<PairCheck>();var tubePairs=new List<PairCheck>();var driver=new List<PairCheck>();
            Stage("Native tube/keepout intersections");
            foreach(var b in structure)foreach(var r in refs)keepout.Add(Intersection(b,r,b.Name,r.Name));
            for(int i=0;i<structure.Length;i++)for(int j=i+1;j<structure.Length;j++)tubePairs.Add(Intersection(structure[i],structure[j],structure[i].Name,structure[j].Name));
            d.ClearSelection2(true);var mass=(MassProperty2)d.Extension.CreateMassProperty2();mass.UseSystemUnits=true;
            mass.IncludeHiddenBodiesOrComponents=true;mass.ShowWeldBeadMass=false;
            mass.SelectedItems=structure.Select(b=>new DispatchWrapper(b)).ToArray();
            bool calculated=mass.Recalculate();
            var selectedNames=Arr(mass.SelectedItems).Cast<Body2>().Select(b=>b.Name).OrderBy(n=>n).ToArray();
            bool selected=calculated&&selectedNames.SequenceEqual(structure.Select(b=>b.Name).OrderBy(n=>n));double volume=structure.Sum(Volume);
            double measuredVolume=mass.Volume,kg=mass.Mass;bool noOverride=!((MassPropertyOverrideOptions)mass.GetOverrideOptions()).OverrideMass;
            bool massOnly=selected&&noOverride&&kg>0&&double.IsFinite(kg)&&Math.Abs(measuredVolume-volume)<Math.Max(1e-12,volume*1e-7);
            string db="";string material=((PartDoc)d).GetMaterialPropertyName2(d.ConfigurationManager.ActiveConfiguration.Name,out db);
            Save(d);string sha=FileSha(snapshot);
            Stage("Native transformed Percy/tube intersections");var p=Open(percy);
            foreach(var c in Arr(((AssemblyDoc)p).GetComponents(false)).Cast<Component2>()) {
                if(c.IsSuppressed()||c.GetModelDoc2() is not ModelDoc2 pd||pd.GetType()!=1)throw new Exception("Missing or suppressed Percy segment");
                foreach(var sourceBody in Bodies(pd)) {
                    var copy=(Body2)sourceBody.Copy();
                    try {
                        var tf=c.GetTotalTransform(true);if(tf!=null&&!copy.ApplyTransform(tf))throw new Exception("Percy body transform failed");
                        foreach(var tube in structure)driver.Add(Intersection(tube,copy,tube.Name,c.Name2));
                    } finally {Marshal.ReleaseComObject(copy);}
                }
            }
            bool Clear(List<PairCheck> checks)=>checks.Count>0&&checks.All(x=>x.resolved&&x.overlapM3<=1e-12);
            var bodies=structure.Select(b=>new {name=b.Name,volumeM3=Volume(b),faultCount=b.Check3?.Count,boundsM=b.GetBodyBox()}).ToArray();
            return new {path,snapshot,cadSha256=sha,sourceCadSha256=FileSha(path),configuration=d.ConfigurationManager.ActiveConfiguration.Name,
                nativeRebuild=rebuilt&&errors.Count==0,featureErrors=errors,bodyValidity=bodies.All(b=>b.faultCount==0&&b.volumeM3>0),
                protectedReferences=SameGeometry(protectedBase,protectedNow),protectedBase,protectedNow,
                bodies,excludedBodies=referenceNames,structuralOnlyMass=massOnly,
                mass=new {kg,weightLb=kg/0.45359237,volumeM3=measuredVolume,expectedVolumeM3=volume,selected,selectedNames,noOverride,material,materialDatabase=db,densityKgM3=mass.Density},
                mainHoopBodies=structure.Count(b=>b.Name.StartsWith("MainHoop_")),mainHoopAudit,
                keepoutClearance=Clear(keepout),driverSolidIntersectionFree=Clear(driver),tubeIntersectionsFree=Clear(tubePairs),
                keepout,tubePairs,driver,
                limitations=new[]{"Single-solid main hoop count alone does not establish continuous manufactured tube compliance.","Percy solid nonintersection is only one necessary check, not full 3D fit, helmet margin or insertion approval.","Native geometric mass does not establish Simulation material or body inclusion.","No beam solve or official score is produced by this audit."}};
        } finally {App.CloseDoc(d.GetTitle());Open(restore);}
    }
}
