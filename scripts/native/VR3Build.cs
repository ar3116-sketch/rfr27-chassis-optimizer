using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    // VR3-style frame build: add each weldment group, immediately cope its tubes onto
    // the structure already present (shop build order, "trim as tubes are added"),
    // save, continue. Bounded batches resume from a per-group journal. A failed group
    // is discarded (the part is closed unsaved and reopened from disk on resume).
    static object BuildTrimFrame(JsonElement job) {
        string output=Safe(job.GetProperty("output").GetString()!);
        string recipePath=job.GetProperty("recipe").GetString()!;
        var recipe=JsonDocument.Parse(File.ReadAllText(recipePath)).RootElement;
        var groups=recipe.GetProperty("groups").EnumerateArray().ToArray();
        var trims=recipe.GetProperty("trims").EnumerateArray().ToArray();
        int start=job.GetProperty("start").GetInt32(),count=job.GetProperty("count").GetInt32();
        bool define=!job.TryGetProperty("defineSketches",out var ds)||ds.GetBoolean();
        string? holder=job.TryGetProperty("restore",out var h)?h.GetString():null;
        string journal=Path.ChangeExtension(output,"build-progress.json");
        if(start<0||count<1||start>=groups.Length)throw new Exception("Bounded group batch required");
        ModelDoc2 d;
        if(start==0) {
            if(File.Exists(output))throw new Exception("Fresh candidate required for group 0");
            if(job.TryGetProperty("source",out var source)){Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(Safe(source.GetString()!),output);d=Open(output);}
            else{d=New(false);d.FeatureManager.InsertWeldmentFeature();SaveNew(d,output);}
        } else {
            var j=JsonDocument.Parse(File.ReadAllText(journal)).RootElement;
            if(j.GetProperty("nextGroup").GetInt32()!=start)throw new Exception("Journal resume index differs from requested start");
            if(j.GetProperty("recipeSha256").GetString()!=FileSha(recipePath))throw new Exception("Recipe changed since the last saved group");
            if(j.GetProperty("cadSha256").GetString()!=FileSha(output))throw new Exception("CAD changed outside the build journal");
            d=FreshOpen(output,holder);
            RestoreBodyIdentities(d,recipe);
        }
        var built=new List<object>();var cuts=new List<object>();int next=start;var sketchStatus=new List<object>();
        try {
            var planes=define?BasePlanesXYZ(d):null;
            using(var batch=new CadBatch(d)) {
                for(int gi=start;gi<Math.Min(groups.Length,start+count);gi++) {
                    var g=groups[gi];string gname=g.GetProperty("name").GetString()!;
                    Stage($"Group {gi+1}/{groups.Length} {gname}");
                    var bodies=AddFrameGroup(d,g,planes,sketchStatus);
                    var names=g.GetProperty("members").EnumerateArray().Select(m=>m.GetProperty("name").GetString()!).ToHashSet();
                    foreach(var t in trims.Where(t=>names.Contains(t.GetProperty("target").GetString()!))) {
                        Stage("  Trim "+t.GetProperty("name").GetString());
                        cuts.Add(CopeFramePair(d,t.GetProperty("target").GetString()!,t.GetProperty("carrier").GetString()!,t.GetProperty("name").GetString()!,
                            t.GetProperty("gapM").GetDouble(),Numbers(t.GetProperty("point")),CarrierMember(recipe,t.GetProperty("carrier").GetString()!),
                            t.TryGetProperty("partialEnd",out var pe)&&pe.GetBoolean(),CarrierMember(recipe,t.GetProperty("target").GetString()!)));
                    }
                    Save(d);next=gi+1;
                    built.Add(new {group=gname,bodies=bodies.Select(ReadBody).ToArray()});
                    File.WriteAllText(journal,JsonSerializer.Serialize(new {path=output,recipe=recipePath,recipeSha256=FileSha(recipePath),cadSha256=FileSha(output),
                        nextGroup=next,groupCount=groups.Length,savedUtc=DateTime.UtcNow},Json));
                }
            }
            bool finished=next>=groups.Length;object? cleanup=null;
            if(finished) {
                cleanup=DeleteSheetBodies(d,"VR3_DeleteCutterSurfaces");
                Mark(d,"RFR_FrameRecipe",recipePath);Mark(d,"RFR_Status","DEVELOPMENT: VR3-order weldment frame; see gate evidence before use");
                Save(d);
                File.WriteAllText(journal,JsonSerializer.Serialize(new {path=output,recipe=recipePath,recipeSha256=FileSha(recipePath),cadSha256=FileSha(output),
                    nextGroup=next,groupCount=groups.Length,finished=true,savedUtc=DateTime.UtcNow},Json));
            }
            object? snapshot=null;
            if(job.TryGetProperty("snapshotDir",out var snapDir))snapshot=SaveChassisImages(d,snapDir.GetString()!,Path.GetFileNameWithoutExtension(output));
            return new {path=output,start,next,groupCount=groups.Length,finished,built,cuts,sketchStatus,cleanup,cadSha256=FileSha(output),snapshot};
        } finally {
            string partPath=d.GetPathName();App.CloseDoc(d.GetTitle());
            if(App.GetOpenDocumentByName(partPath)!=null)Stage("WARNING: part still loaded after close");
            if(holder!=null&&File.Exists(Safe(holder)))Open(holder);
        }
    }

    // Recovery after a native job was killed mid-command: leave any active tool or sketch
    // and close the part WITHOUT saving (the journal points at the last saved group).
    static object RecoverPart(JsonElement job) {
        string path=Safe(job.GetProperty("path").GetString()!);
        if(App.GetOpenDocumentByName(path) is not ModelDoc2 d)return new {path,wasOpen=false};
        bool dirty=d.GetSaveFlag();
        try{d.SetPickMode();d.ClearSelection2(true);if(d.SketchManager.ActiveSketch!=null)d.SketchManager.Insert3DSketch(true);}
        catch(Exception x){Stage("Recover: "+x.Message);}
        App.CloseDoc(d.GetTitle());
        return new {path,wasOpen=true,unsavedChangesDiscarded=dirty,stillOpen=App.GetOpenDocumentByName(path)!=null};
    }

    // After a reopen, cut features may have renamed tube bodies ("Cope_<tube>_to_...").
    static void RestoreBodyIdentities(ModelDoc2 d,JsonElement recipe) {
        foreach(var m in recipe.GetProperty("groups").EnumerateArray().SelectMany(g=>g.GetProperty("members").EnumerateArray())) {
            string n=m.GetProperty("name").GetString()!;
            if(Bodies(d).Any(b=>b.Name==n))continue;
            var renamed=Bodies(d).Where(b=>b.Name.StartsWith("Cope_"+n+"_to_",StringComparison.Ordinal)).ToArray();
            if(renamed.Length==0)continue;  // not built yet in this batch sequence
            if(renamed.Length!=1)throw new Exception("Ambiguous semantic body "+n);
            var f=Find(d,renamed[0].Name)??throw new Exception("Missing combine identity feature");
            if(f.GetTypeName2() is not ("CombineBodies" or "SurfCut"))throw new Exception("Unexpected renamed body feature "+f.GetTypeName2());
            var previous=Find(d,n);if(previous!=null&&previous.Name!=f.Name)previous.Name="IdentityHistory_"+f.Name;
            f.Name=n;renamed[0].Name=n;
        }
    }

    // Remove the trims' temporary cutting surfaces with one Delete Body feature at the
    // end of the tree. Earlier cut features keep their results.
    static object DeleteSheetBodies(ModelDoc2 d,string featureName) {
        var solids=Bodies(d).ToDictionary(b=>b.Name,Volume);
        var sheets=Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Cast<Body2>().ToArray();
        if(sheets.Length==0)return new {deleted=0};
        d.ClearSelection2(true);
        for(int i=0;i<sheets.Length;i++)if(!sheets[i].Select2(i>0,null))throw new Exception("Select cutter surface "+sheets[i].Name);
        var del=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Delete cutter surfaces failed");del.Name=featureName;
        d.ClearSelection2(true);if(!d.EditRebuild3()||del.GetErrorCode()!=0)throw new Exception("Cutter surface cleanup rebuild");
        if(Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Length!=0)throw new Exception("Cutter surfaces remain");
        var after=Bodies(d);
        if(after.Length!=solids.Count||after.Any(b=>!solids.ContainsKey(b.Name)||Math.Abs(solids[b.Name]-Volume(b))>1e-12))throw new Exception("Solid tubes changed during surface cleanup");
        return new {deleted=sheets.Length,solidsUnchanged=after.Length};
    }

    // The part's base planes, ordered by normal X, Y, Z (planes through the origin).
    static Feature[] BasePlanesXYZ(ModelDoc2 d) {
        var found=new Feature?[3];
        for(var f=(Feature?)d.FirstFeature();f!=null;f=(Feature?)f.GetNextFeature()) {
            if(f.GetTypeName2()!="RefPlane")continue;
            var t=(double[])((RefPlane)f.GetSpecificFeature2()).Transform.ArrayData;
            var n=new[]{t[6],t[7],t[8]};var o=new[]{t[9],t[10],t[11]};
            if(Math.Abs(Dot(n,o))>1e-9)continue;
            for(int k=0;k<3;k++)if(found[k]==null&&Math.Abs(Math.Abs(n[k])-1)<1e-9)found[k]=f;
        }
        if(found.Any(x=>x==null))throw new Exception("Three orthogonal base planes through the origin are required");
        return found!;
    }

    // Fully define a group's 3D sketch without Fix: every distinct endpoint gets three
    // driving distances to the X/Y/Z base planes (or a coincident relation when it lies
    // on one); duplicate points at the same location are made coincident.
    static int DefineSketchPoints(ModelDoc2 d,IEnumerable<SketchSegment> lines,Feature[] planes) {
        var pts=lines.Cast<SketchLine>().SelectMany(l=>new[]{(SketchPoint)l.GetStartPoint2(),(SketchPoint)l.GetEndPoint2()})
            .GroupBy(p=>string.Join(",",((int[])p.GetID()).Select(x=>x.ToString()))).Select(g=>g.First()).ToList();
        // No interactive prompts while the API dimensions the sketch: a hidden "make this
        // dimension driven?" prompt or inference relations blocked a full-frame build.
        var toggles=new (swUserPreferenceToggle_e key,bool value)[]{
            (swUserPreferenceToggle_e.swInputDimValOnCreate,false),
            (swUserPreferenceToggle_e.swSketchOverdefiningDimsPromptToSetState,false),
            (swUserPreferenceToggle_e.swSketchOverdefiningDimsSetDrivenByDefault,true),
            (swUserPreferenceToggle_e.swSketchAutomaticRelations,false),
            (swUserPreferenceToggle_e.swSketchInference,false)};
        var saved=toggles.Select(t=>App.GetUserPreferenceToggle((int)t.key)).ToArray();
        foreach(var t in toggles)App.SetUserPreferenceToggle((int)t.key,t.value);
        int dims=0,specific=0,legacy=0;
        try {
            var masters=new List<SketchPoint>();
            foreach(var p in pts) {
                var xyz=new[]{p.X,p.Y,p.Z};
                var master=masters.FirstOrDefault(q=>Math.Abs(q.X-xyz[0])<1e-9&&Math.Abs(q.Y-xyz[1])<1e-9&&Math.Abs(q.Z-xyz[2])<1e-9);
                if(master!=null) {
                    d.ClearSelection2(true);if(!p.Select4(false,null)||!master.Select4(true,null))throw new Exception("Select duplicate sketch points");
                    d.SketchAddConstraints("sgCOINCIDENT");continue;
                }
                masters.Add(p);
                for(int k=0;k<3;k++) {
                    d.ClearSelection2(true);
                    if(!p.Select4(false,null)||!planes[k].Select2(true,0))throw new Exception("Select point and base plane");
                    int selected=((SelectionMgr)d.SelectionManager).GetSelectedObjectCount2(-1);
                    if(selected!=2)throw new Exception($"Point/plane selection holds {selected} items");
                    if(Math.Abs(xyz[k])<1e-9){d.SketchAddConstraints("sgCOINCIDENT");continue;}
                    var at=new[]{xyz[0],xyz[1],xyz[2]};at[k]/=2;
                    // AddSpecificDimension reports an error instead of falling back to the
                    // interactive Smart Dimension tool (AddDimension2 can wait there for clicks).
                    // Prefer AddSpecificDimension (returns an error instead of entering the
                    // interactive Smart Dimension tool, where AddDimension2 can wait for clicks).
                    // Accept it only if it measures this point's distance to the plane.
                    object? dim=null;
                    foreach(var ty in new[]{(int)swDimensionType_e.swHorLinearDimension,(int)swDimensionType_e.swVertLinearDimension}) {
                        int er=0;
                        if(d.Extension.AddSpecificDimension(at[0],at[1],at[2],ty,ref er) is DisplayDimension tried) {
                            double v=Math.Abs(((Dimension)tried.GetDimension2(0)).SystemValue);
                            if(Math.Abs(v-Math.Abs(xyz[k]))<1e-7){dim=tried;specific++;break;}
                            d.ClearSelection2(true);((Annotation)tried.GetAnnotation()).Select3(false,null);d.EditDelete();
                        }
                        d.ClearSelection2(true);
                        if(!p.Select4(false,null)||!planes[k].Select2(true,0))throw new Exception("Reselect point and base plane");
                    }
                    if(dim==null)legacy++;
                    dim??=d.AddDimension2(at[0],at[1],at[2]);
                    if(dim is not DisplayDimension)throw new Exception($"Point-to-plane dimension failed at {string.Join(",",xyz)}");
                    dims++;
                }
            }
        } finally {
            for(int i=0;i<toggles.Length;i++)App.SetUserPreferenceToggle((int)toggles[i].key,saved[i]);
            d.ClearSelection2(true);
        }
        Stage($"Sketch dimensions: {specific} non-interactive, {legacy} legacy");
        return dims;
    }
}
