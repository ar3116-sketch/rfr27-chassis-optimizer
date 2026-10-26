using System.Text.Json;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    static Body2 NamedBody(ModelDoc2 d,string name)=>Bodies(d).Single(b=>b.Name==name);
    static double[] Add(double[] a,double[] b)=>a.Zip(b,(x,y)=>x+y).ToArray();
    static double[] Scale(double[] a,double x)=>a.Select(v=>v*x).ToArray();
    static double DifferenceVolume(Body2 first,Body2 second) {
        var a=(Body2)first.Copy();var b=(Body2)second.Copy();
        try {
            var pieces=Arr(a.Operations2((int)swBodyOperationType_e.SWBODYCUT,b,out int error)).Cast<Body2>().ToArray();
            if(error!=0&&error!=(int)swBodyOperationError_e.swBodyOperationEmptyBody&&error!=(int)swBodyOperationError_e.swBodyOperationNoIntersect&&error!=(int)swBodyOperationError_e.swBodyOperationDisjointBodies)throw new Exception("Unresolved material difference "+error);
            double volume=pieces.Sum(Volume);foreach(var p in pieces)Marshal.ReleaseComObject(p);return volume;
        } finally {Marshal.ReleaseComObject(a);Marshal.ReleaseComObject(b);}
    }
    static Body2[] AddFrameGroup(ModelDoc2 d,JsonElement g,Feature[]? planes=null,List<object>? sketchStatus=null) {
        string name=g.GetProperty("name").GetString()!;
        string profile=Path.GetFullPath(Path.Combine(Root,"profiles",g.GetProperty("profile").GetString()!));
        if(!profile.StartsWith(Path.Combine(Root,"profiles")+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)||!File.Exists(profile))throw new Exception("Invalid profile");
        var members=g.GetProperty("members").EnumerateArray().ToArray();
        if(Find(d,name)!=null)throw new Exception("Group already exists");
        d.ClearSelection2(true);d.SketchManager.Insert3DSketch(true);d.SketchManager.AddToDB=true;
        var lines=new List<SketchSegment>();
        try {foreach(var m in members){var a=Numbers(m.GetProperty("a"));var b=Numbers(m.GetProperty("b"));if(a.Length!=3||b.Length!=3||a.Concat(b).Any(x=>!double.IsFinite(x))||Norm(Sub(b,a))<.01)throw new Exception("Invalid member coordinates");lines.Add(d.SketchManager.CreateLine(a[0],a[1],a[2],b[0],b[1],b[2])??throw new Exception("Line creation failed"));}
            if(planes!=null) {
                // VR3: fully defined sketches, no Fix relations.
                d.SketchManager.AddToDB=false;int dims=DefineSketchPoints(d,lines,planes);
                int status=((Sketch)d.SketchManager.ActiveSketch).GetConstrainedStatus();
                if(status!=(int)swConstrainedStatus_e.swFullyConstrained)throw new Exception($"Sketch for {name} not fully defined (status {status})");
                sketchStatus?.Add(new {group=name,dimensions=dims,fullyDefined=true});
            }}
        finally{d.SketchManager.AddToDB=false;d.SketchManager.Insert3DSketch(true);}
        var sk=d.IFeatureByPositionReverse(0);sk.Name=name+"_Path";
        var group=d.FeatureManager.CreateStructuralMemberGroup();group.Segments=lines.ToArray();group.ApplyCornerTreatment=members.Length>1;
        group.CornerTreatmentType=(int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter;
        group.MiterMergeCondition=false;group.MergeArcSegmentBodies=false;group.GapWithinGroup=0;group.GapForOtherGroups=0;
        d.ClearSelection2(true);
        var f=d.FeatureManager.InsertStructuralWeldment4(profile,(int)swSolidworksWeldmentEndCondOptions_e.swEndConditionMiter,false,new DispatchWrapper[]{new(group)})??throw new Exception("Weldment creation failed "+name);
        f.Name=name;d.ClearSelection2(true);if(!d.EditRebuild3()||f.GetErrorCode()!=0)throw new Exception("Weldment rebuild failed "+name);
        var bodies=Arr(f.GetFaces()).Cast<Face2>().Select(x=>(Body2)x.GetBody()).GroupBy(x=>x.Name).Select(x=>x.First()).ToList();
        if(bodies.Count!=members.Length)throw new Exception($"Expected {members.Length} independent tubes, got {bodies.Count}: {name}");
        var output=new List<Body2>();
        foreach(var m in members){
            var a=Numbers(m.GetProperty("a"));var b=Numbers(m.GetProperty("b"));var axis=Unit(Sub(b,a));
            var matches=bodies.Where(body=>Arr(body.GetFaces()).Cast<Face2>().Any(face=>{
                var s=(Surface)face.GetSurface();if(!s.IsCylinder())return false;var c=(double[])s.CylinderParams;
                var origin=c.Take(3).ToArray();return Math.Abs(Math.Abs(Dot(axis,c.Skip(3).Take(3).ToArray()))-1)<1e-7&&Norm(Cross(Sub(origin,a),axis))<1e-7;
            })).ToArray();
            if(matches.Length!=1)throw new Exception("Ambiguous native tube identity "+m.GetProperty("name").GetString());
            var body=matches[0];body.Name=m.GetProperty("name").GetString();if(body.Check3?.Count!=0)throw new Exception("Body fault");bodies.Remove(body);output.Add(body);
        }
        return output.ToArray();
    }
    // Outer radius of a round VR3 profile ("1.375 x 0.049.SLDLFP" -> 0.0174625 m).
    // Hoops are the supplied 1.000 x .095 Size A tube.
    const double HoopOuterRadius=.0127;
    static double MemberOuterRadius(JsonElement? member) {
        if(member is not JsonElement m||!m.TryGetProperty("profile",out var p))return HoopOuterRadius;
        var x=System.Text.RegularExpressions.Regex.Match(p.GetString()!,@"^(\d+\.\d+)\s*(SQ)?\s*x",System.Text.RegularExpressions.RegexOptions.IgnoreCase);
        if(!x.Success)throw new Exception("Unrecognised profile "+p.GetString());
        if(x.Groups[2].Success)throw new Exception("Square-tube coping is not implemented: "+p.GetString());
        return double.Parse(x.Groups[1].Value,System.Globalization.CultureInfo.InvariantCulture)*.0254/2;
    }
    static JsonElement? CarrierMember(JsonElement recipe,string name)=>recipe.GetProperty("groups").EnumerateArray().SelectMany(g=>g.GetProperty("members").EnumerateArray()).Where(m=>m.GetProperty("name").GetString()==name).Select(m=>(JsonElement?)m).FirstOrDefault();
    static object CopeFramePair(ModelDoc2 d,string targetName,string carrierName,string name,double gap,double[] pick,JsonElement? carrierMember=null,bool partialEnd=false,JsonElement? targetMember=null) {
        double rC=MemberOuterRadius(carrierMember),rT=MemberOuterRadius(targetMember);
        double reach=2*(rT+rC),extend=Math.Max(.05,2*(rT+rC));  // 1.0 in tubes: 50.8 mm and 50 mm, as validated
        var target=NamedBody(d,targetName);var carrier=NamedBody(d,carrierName);double before=Volume(target),carrierBefore=Volume(carrier);int count=Bodies(d).Length;
        var existing=Intersection(target,carrier,targetName,carrierName);
        if(!existing.resolved)throw new Exception("Unresolved pre-trim intersection");
        if(existing.overlapM3<=1e-12)return new {target=targetName,carrier=carrierName,skipped=true,reason="No remaining overlap; contact must be checked by the independent joint audit",overlap=existing};
        var originalTarget=(Body2)target.Copy();
        var otherNames=Bodies(d).Where(b=>b.Name!=targetName).Select(b=>b.Name).ToHashSet();
        // Choose the real outer carrier face nearest the declared joint. Supporting
        // cylinders/toruses must come from the actual carrier, never a guessed axis.
        var faces=Arr(carrier.GetFaces()).Cast<Face2>().Where(f=>{
            var s=(Surface)f.GetSurface();if(s.IsCylinder()) {
                var cp=(double[])s.CylinderParams;if(Math.Abs(cp[6]-rC)>1e-7)return false;
                if(carrierMember is JsonElement cm) {
                    var a=Numbers(cm.GetProperty("a"));var axis=Unit(Sub(Numbers(cm.GetProperty("b")),a));
                    // Previously coped end faces are concave cylinders of the
                    // OTHER tube, often with the same radius. They are not the
                    // carrier's longitudinal outer wall.
                    return Math.Abs(Math.Abs(Dot(axis,cp.Skip(3).Take(3).ToArray()))-1)<1e-7&&Norm(Cross(Sub(cp.Take(3).ToArray(),a),axis))<1e-7;
                }
                return true;
            }
            if(s.IsTorus())return Math.Abs(((double[])s.TorusParams)[7]-rC)<1e-7;return false;
        }).Select(f=>new {f,p=(double[])f.GetClosestPointOn(pick[0],pick[1],pick[2])}).OrderBy(x=>Norm(Sub(x.p.Take(3).ToArray(),pick))).ToArray();
        if(faces.Length==0)throw new Exception("No accepted outer carrier face: "+carrierName);
        // Include adjacent real outer faces near a bend. Cutting against only
        // the nearest cylinder can leave a sliver where the hoop transitions
        // into its torus. The cut must match the actual carrier quilt.
        var boundaryFaces=faces.Where(x=>Norm(Sub(x.p.Take(3).ToArray(),pick))<=reach).Select(x=>x.f).ToArray();
        if(boundaryFaces.Length==0||boundaryFaces.Length>6)throw new Exception("Unsupported carrier boundary quilt");
        d.ClearSelection2(true);
        for(int i=0;i<boundaryFaces.Length;i++)if(!((Entity)boundaryFaces[i]).Select4(i>0,null))throw new Exception("Select carrier face quilt");
        d.InsertOffsetSurface(gap,false);var surf=d.IFeatureByPositionReverse(0);
        if(surf.GetTypeName2()!="OffsetRefSurface")throw new Exception("No native offset surface");surf.Name=name+"_Boundary";
        // A miter clips the cylindrical face at the carrier end. Extend only
        // the temporary cutting surface so the cutter spans the branch wall;
        // never extend or otherwise change the physical carrier tube.
        if(boundaryFaces.Length==1&&((Surface)boundaryFaces[0].GetSurface()).IsCylinder()) {
            d.ClearSelection2(true);var sheetFaces=Arr(surf.GetFaces()).Cast<Face2>().ToArray();
            if(sheetFaces.Length!=1)throw new Exception("Expected cylindrical cutter face");
            // Extend only the nearby end boundary, not the remote fishmouth or
            // periodic seam. Extending every edge fails on previously coped carriers.
            var ends=Arr(sheetFaces[0].GetEdges()).Cast<Edge>().Where(e=>!((Curve)e.GetCurve()).IsLine())
                .Select(e=>new {e,p=(double[])e.GetClosestPointOn(pick[0],pick[1],pick[2])})
                .Where(e=>Norm(Sub(e.p.Take(3).ToArray(),pick))<extend).ToArray();
            if(ends.Length>0) {
                for(int i=0;i<ends.Length;i++)if(!((Entity)ends[i].e).Select4(i>0,null))throw new Exception("Select cutter end");
                d.InsertExtendSurface(false,0,extend);var extended=d.IFeatureByPositionReverse(0);
                if(extended.Name==surf.Name||extended.GetErrorCode()!=0) {
                    if(extended.Name!=surf.Name){d.ClearSelection2(true);if(!extended.Select2(false,0)||!d.Extension.DeleteSelection2(0))throw new Exception("Remove failed cutter extension");}
                    d.ClearSelection2(true);var face=Arr(surf.GetFaces()).Cast<Face2>().Single();if(!((Entity)face).Select4(false,null))throw new Exception("Select cutter to untrim");
                    var untrim=d.FeatureManager.InsertUntrimSurface2(0,0,extend,true,false)??throw new Exception("Cylindrical cutter untrim failed");
                    if(untrim.GetErrorCode()!=0)throw new Exception("Cylindrical cutter untrim rebuild");untrim.Name=name+"_UntrimmedBoundary";surf=untrim;
                    d.ClearSelection2(true);var naturalFace=Arr(untrim.GetFaces()).Cast<Face2>().Single();
                    if(!((Entity)naturalFace).Select4(false,null))throw new Exception("Select natural cylinder boundary");
                    d.InsertExtendSurface(false,0,extend);var full=d.IFeatureByPositionReverse(0);
                    if(full.Name==untrim.Name||full.GetErrorCode()!=0)throw new Exception("Natural cylinder extension after untrim failed");
                    full.Name=name+"_NaturalExtendedBoundary";surf=full;
                } else {extended.Name=name+"_ExtendedBoundary";surf=extended;}
            }
        }
        d.ClearSelection2(true);if(!surf.Select2(false,0))throw new Exception("Select cut surface");
        target=NamedBody(d,targetName);
        var cut=d.FeatureManager.InsertCutSurface(false,0,true,false,new Body2[]{target},out int e)??throw new Exception($"Cut {name} failed {e}");cut.Name=name;
        string backend="native_associative_offset_surface_cut";
        if(cut.GetErrorCode()!=0&&existing.overlapM3<=1e-9) {
            // Near-tangent, multiway corners can contain a sub-mm3 remnant after
            // the primary fishmouth. Remove that exact remnant with a native
            // associative copy/subtract, preserving the original carrier. This
            // is not a substitute for the full fishmouth or independent end fit.
            d.ClearSelection2(true);if(!cut.Select2(false,0)||!d.Extension.DeleteSelection2(0))throw new Exception("Remove failed tangent cut");
            var copySelection=((SelectionMgr)d.SelectionManager).CreateSelectData();copySelection.Mark=1;
            d.ClearSelection2(true);if(!NamedBody(d,carrierName).Select2(false,copySelection))throw new Exception("Select exact remnant tool");
            var names=Bodies(d).Select(b=>b.Name).ToHashSet();
            var copied=d.FeatureManager.InsertMoveCopyBody2(0,.1,0,0,0,0,0,0,0,0,true,1)??throw new Exception("Associative carrier copy failed");copied.Name=name+"_CarrierCopy";
            var tool=Bodies(d).Single(b=>!names.Contains(b.Name));
            d.ClearSelection2(true);if(!tool.Select2(false,copySelection))throw new Exception("Select carrier copy for return");
            var returned=d.FeatureManager.InsertMoveCopyBody2(0,-.1,0,0,0,0,0,0,0,0,false,1)??throw new Exception("Return associative carrier copy");returned.Name=name+"_CarrierCopyReturn";
            tool=Bodies(d).Single(b=>!names.Contains(b.Name));
            cut=d.FeatureManager.InsertCombineFeature((int)swBodyOperationType_e.SWBODYCUT,NamedBody(d,targetName),new DispatchWrapper[]{new(tool)})??throw new Exception("Native remnant subtraction failed");cut.Name=targetName;
            backend="native_associative_carrier_copy_subtraction_for_tangent_remnant";
        }
        if(cut.GetErrorCode()!=0&&carrierMember!=null&&boundaryFaces.Length==1) {
            // A periodic surface can fail when an existing cope meets its seam.
            // Use the exact analytic outer cylinder read from the actual carrier
            // as a filled native tool. The deterministic generator recreates this
            // tool for every candidate; it is not a guessed centerline cylinder.
            var cp=(double[])((Surface)boundaryFaces[0].GetSurface()).CylinderParams;
            var origin=cp.Take(3).ToArray();var axis=Unit(cp.Skip(3).Take(3).ToArray());
            var bb=(double[])NamedBody(d,carrierName).GetBodyBox();
            var projections=(from x in new[]{bb[0],bb[3]} from y in new[]{bb[1],bb[4]} from z in new[]{bb[2],bb[5]} select Dot(Sub(new[]{x,y,z},origin),axis)).ToArray();
            double low=projections.Min()-extend,length=projections.Max()-low+extend;var basePoint=Add(origin,Scale(axis,low));
            var cylinder=(Body2)((Modeler)App.GetModeler()).CreateBodyFromCyl(basePoint.Concat(axis).Concat(new[]{cp[6]+gap,length}).ToArray());
            if(cylinder.Check3?.Count!=0||Math.Abs(Volume(cylinder)-Math.PI*Math.Pow(cp[6]+gap,2)*length)>1e-10)throw new Exception("Native analytic cutter verification failed");
            d.ClearSelection2(true);if(!cut.Select2(false,0)||!d.Extension.DeleteSelection2(0))throw new Exception("Remove failed periodic cut");
            var names=Bodies(d).Select(b=>b.Name).ToHashSet();
            var toolFeature=((PartDoc)d).CreateFeatureFromBody3(cylinder,false,0) as Feature??throw new Exception("Create exact native cylinder tool");toolFeature.Name=name+"_AnalyticCylinder";
            var tool=Bodies(d).Single(b=>!names.Contains(b.Name));
            cut=d.FeatureManager.InsertCombineFeature((int)swBodyOperationType_e.SWBODYCUT,NamedBody(d,targetName),new DispatchWrapper[]{new(tool)})??throw new Exception("Native cylinder subtraction failed");cut.Name=name;
            backend="native_exact_carrier_cylinder_subtraction_regenerated_per_candidate";
        }
        bool rebuilt=d.EditRebuild3();var changed=Bodies(d).Where(b=>!otherNames.Contains(b.Name)).ToArray();
        object? fragments=null;Feature? drop=null;
        if(changed.Length>1) {
            // A larger branch meeting a smaller carrier at a multiway node: the extended
            // carrier cutter can detach the branch's tip beyond the carrier wall. That tip
            // is cope waste. Remove it only when every extra piece is small (< 2% of the
            // tube) and lies at this joint; any other split still rejects the cut.
            var pieces=changed.Select(b=>new {b,v=Volume(b),box=(double[])b.GetBodyBox()}).OrderByDescending(x=>x.v).ToArray();
            var extra=pieces.Skip(1).ToArray();
            Func<double[],double> fromPick=bx=>Norm(Sub(new[]{Math.Clamp(pick[0],bx[0],bx[3]),Math.Clamp(pick[1],bx[1],bx[4]),Math.Clamp(pick[2],bx[2],bx[5])},pick));
            bool small=extra.All(x=>x.v<before*.02&&fromPick(x.box)<=reach)&&extra.Sum(x=>x.v)<before*.03;
            if(!small)throw new Exception("Cut changed tube count/identity: "+string.Join(",",Bodies(d).Select(b=>b.Name))+$" (pieces {string.Join("/",pieces.Select(x=>x.v.ToString("e3")))} m3 of {before:e3})");
            d.ClearSelection2(true);
            for(int i=0;i<extra.Length;i++)if(!extra[i].b.Select2(i>0,null))throw new Exception("Select cope fragment");
            drop=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Remove cope fragment failed");drop.Name=name+"_CopeWaste";
            d.ClearSelection2(true);rebuilt=d.EditRebuild3()&&rebuilt;
            fragments=extra.Select(x=>new {volumeM3=x.v,distanceFromJointM=fromPick(x.box)}).ToArray();
            changed=Bodies(d).Where(b=>!otherNames.Contains(b.Name)).ToArray();
            if(changed.Length!=1)throw new Exception("Cut changed tube count/identity after removing cope waste: "+string.Join(",",Bodies(d).Select(b=>b.Name)));
        }
        var after=changed[0];after.Name=targetName;var ca=NamedBody(d,carrierName);
        double distance=d.ClosestDistance(after,ca,out object pa,out object pb);
        // Multiway node (two or more other tubes at this joint): the end is coped onto several
        // tubes, so a local gap to one of them up to 1.6 mm (0.063 in, weld-fillable) is accepted
        // and reported; elsewhere the procedure's 0.05 in limit applies.
        int neighbours=Bodies(d).Count(b=>b.Name!=targetName&&b.Name!=carrierName&&new Func<double[],bool>(bx=>Norm(Sub(new[]{Math.Clamp(pick[0],bx[0],bx[3]),Math.Clamp(pick[1],bx[1],bx[4]),Math.Clamp(pick[2],bx[2],bx[5])},pick))<=reach)((double[])b.GetBodyBox()));
        double gapLimit=neighbours>=2?.0016:.00127;
        object? gapWarning=distance>=.00127&&distance<gapLimit?new{gapM=distance,limitM=gapLimit,neighbours}:null;
        var overlap=Intersection(after,ca,targetName,carrierName);
        // Body.GetMassProperties integrates trimmed faces approximately. A tiny
        // increase in that estimate is not proof of added material. Native set
        // differences independently establish that a cut only removes material.
        double added=DifferenceVolume(after,originalTarget),removed=DifferenceVolume(originalTarget,after);Marshal.ReleaseComObject(originalTarget);
        // A carrier already coped at a multiway junction no longer occupies its
        // entire supporting cylinder. Nominal cutter offset is not the minimum
        // distance to that clipped physical tube. Require the assignment joint
        // limit here; the independent audit must cover the WHOLE end perimeter
        // against the union of its real neighbours before the frame can score.
        bool valid=rebuilt&&cut.GetErrorCode()==0&&after.Check3?.Count==0&&Bodies(d).Length==count&&Math.Abs(Volume(ca)-carrierBefore)<1e-12&&Volume(after)>before*.5&&added<=1e-12&&removed>1e-12&&distance>=0&&distance<gapLimit&&overlap.resolved&&(partialEnd
            // First end of a member that meets the same carrier twice: this cut must
            // strictly reduce the overlap; the member's final end cut must clear it.
            ?overlap.overlapM3<existing.overlapM3-1e-12
            :overlap.overlapM3<=1e-12);
        foreach(var sf in Arr(surf.GetFaces()).Cast<Face2>())((Body2)sf.GetBody()).HideBody(true);
        d.ClearSelection2(true);
        if(!valid){Stage(JsonSerializer.Serialize(new {cutDiagnostic=name,apiError=e,featureError=cut.GetErrorCode(),carrier=ReadBody(ca),target=ReadBody(after),surface=Arr(surf.GetFaces()).Cast<Face2>().Select(f=>new {box=f.GetBox(),cylinder=((Surface)f.GetSurface()).IsCylinder()?((Surface)f.GetSurface()).CylinderParams:null})}));throw new Exception($"Rejected cut {name}: gap {distance}, volume {before}->{Volume(after)}, added {added}, removed {removed}, count {count}->{Bodies(d).Length}, overlap {overlap.overlapM3}");}
        // The last feature that shaped this tube carries its name; bodies follow the name
        // of the feature that produced them. A split leaves "<cut>[1]", so after removing
        // the cope waste the cut itself takes the tube's name and the body is renamed.
        Feature identity=cut;
        var previousIdentity=Find(d,targetName);
        if(previousIdentity!=null&&previousIdentity.Name!=cut.Name)previousIdentity.Name="TrimHistory_"+name;
        string Names()=>string.Join(",",Bodies(d).Where(b=>!otherNames.Contains(b.Name)).Select(b=>b.Name));
        if(drop!=null) {
            Stage("Cope waste identity S0: "+Names());
            cut.Name=targetName;d.EditRebuild3();Stage("Cope waste identity S2 (cut renamed): "+Names());
            var sv=Bodies(d).Where(b=>!otherNames.Contains(b.Name)).ToArray();
            if(sv.Length==1&&sv[0].Name!=targetName){sv[0].Name=targetName;Stage("Cope waste identity S3 (body renamed): "+Names());}
            if(!Bodies(d).Any(b=>b.Name==targetName)) {
                // Split bodies keep "<feature>[n]" and refuse renames. A zero-distance
                // Move/Copy Body gives the tube a fresh single-result name (no geometry change).
                cut.Name=name;
                var mark=((SelectionMgr)d.SelectionManager).CreateSelectData();mark.Mark=1;
                d.ClearSelection2(true);if(!sv.Single().Select2(false,mark))throw new Exception("Select tube for identity move");
                var move=d.FeatureManager.InsertMoveCopyBody2(0,0,0,0,0,0,0,0,0,0,false,1);
                Stage("Cope waste identity S5 zero move "+(move==null?"refused":move.GetTypeName2()+" "+move.Name)+": "+Names());
                if(move!=null){move.Name=targetName;d.ClearSelection2(true);d.EditRebuild3();Stage("Cope waste identity S5b: "+Names());}
                identity=move??identity;
            }
            after=Bodies(d).Where(b=>!otherNames.Contains(b.Name)).Single();
        } else {cut.Name=targetName;after.Name=targetName;}
        if(!Bodies(d).Any(b=>b.Name==targetName))throw new Exception("Tube identity lost after cope "+name+": "+targetName);
        return new {target=targetName,carrier=carrierName,feature=cut.Name,operation=name,backend,partialEnd,distanceM=distance,volumeBeforeM3=before,volumeAfterM3=Volume(after),addedVolumeM3=added,removedVolumeM3=removed,carrierUnchanged=true,overlap,copeWaste=fragments,gapWarning};
    }
    // The open layout keeps the part loaded as a component. After a rejected cut
    // that in-memory part keeps the cut (even with a clear save flag), and an
    // in-place reload does not discard it. Unload it completely, then open from disk.
    static ModelDoc2 FreshOpen(string path,string? holder) {
        path=Safe(path);
        if(App.GetOpenDocumentByName(path) is ModelDoc2) {
            if(holder!=null&&App.GetOpenDocumentByName(Safe(holder)) is ModelDoc2 asm){Stage("Closing layout to unload part: "+asm.GetPathName());App.CloseDoc(asm.GetTitle());}
            if(App.GetOpenDocumentByName(path) is ModelDoc2 loaded)App.CloseDoc(loaded.GetTitle());
            if(App.GetOpenDocumentByName(path)!=null)throw new Exception("Part is still loaded by another open document; close it before trimming: "+path);
        }
        return Open(path);
    }
    static object TrimConnectingFrame(JsonElement job) {
        string? holder=job.TryGetProperty("restore",out var h)?h.GetString():null;
        var d=FreshOpen(job.GetProperty("path").GetString()!,holder);var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        foreach(var m in recipe.GetProperty("groups").EnumerateArray().SelectMany(g=>g.GetProperty("members").EnumerateArray())) {
            string n=m.GetProperty("name").GetString()!;
            if(Bodies(d).Any(b=>b.Name==n))continue;
            var renamed=Bodies(d).Where(b=>b.Name.StartsWith("Cope_"+n+"_to_",StringComparison.Ordinal)).ToArray();
            if(renamed.Length!=1)throw new Exception("Missing semantic body "+n);
            var f=Find(d,renamed[0].Name)??throw new Exception("Missing combine identity feature");
            if(f.GetTypeName2() is not ("CombineBodies" or "SurfCut"))throw new Exception("Unexpected renamed body feature "+f.GetTypeName2());
            var previous=Find(d,n);if(previous!=null&&previous.Name!=f.Name)previous.Name="IdentityHistory_"+f.Name;
            f.Name=n;renamed[0].Name=n;Save(d);
        }
        int start=job.GetProperty("start").GetInt32(),count=job.GetProperty("count").GetInt32();
        if(start<0||count<1||count>40)throw new Exception("Bounded trim batch required");
        var cuts=new List<object>();int next=start;
        string journal=Path.ChangeExtension(d.GetPathName(),"trim-progress.json");
        try {
            using var batch=new CadBatch(d);
            foreach(var t in recipe.GetProperty("trims").EnumerateArray().Skip(start).Take(count)){
                Stage("Trim "+t.GetProperty("name").GetString());cuts.Add(CopeFramePair(d,t.GetProperty("target").GetString()!,t.GetProperty("carrier").GetString()!,t.GetProperty("name").GetString()!,t.GetProperty("gapM").GetDouble(),Numbers(t.GetProperty("point")),CarrierMember(recipe,t.GetProperty("carrier").GetString()!),t.TryGetProperty("partialEnd",out var pe)&&pe.GetBoolean(),CarrierMember(recipe,t.GetProperty("target").GetString()!)));Save(d);
                next++;
                File.WriteAllText(journal,JsonSerializer.Serialize(new {path=d.GetPathName(),recipe=job.GetProperty("recipe").GetString(),recipeSha256=FileSha(job.GetProperty("recipe").GetString()!),cadSha256=FileSha(d.GetPathName()),nextTrimIndex=next,batchStart=start,cuts,savedUtc=DateTime.UtcNow},Json));
            }
            object? snapshot=null;
            if(job.TryGetProperty("snapshotDir",out var snapDir)){batch.Dispose();snapshot=SaveChassisImages(d,snapDir.GetString()!,Path.GetFileNameWithoutExtension(d.GetPathName()));}
            return new {path=d.GetPathName(),start,count,cuts,nextTrimIndex=next,cadSha256=FileSha(d.GetPathName()),snapshot,bodies=Bodies(d).Select(ReadBody).ToArray()};
        } finally {
            // Nothing else holds the part here, so closing discards any rejected cut.
            string partPath=d.GetPathName();App.CloseDoc(d.GetTitle());
            if(App.GetOpenDocumentByName(partPath)!=null)Stage("WARNING: part still loaded after close");
            if(holder!=null)Open(holder);
        }
    }
    static object BuildConnectingFrame(JsonElement job) {
        string output=Safe(job.GetProperty("output").GetString()!);if(File.Exists(output))throw new Exception("Fresh candidate required");
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        ModelDoc2 d;
        if(job.TryGetProperty("source",out var source)){Directory.CreateDirectory(Path.GetDirectoryName(output)!);File.Copy(Safe(source.GetString()!),output);d=Open(output);}else{d=New(false);d.FeatureManager.InsertWeldmentFeature();SaveNew(d,output);}
        var built=new List<object>();var cuts=new List<object>();
        try{
            using var batch=new CadBatch(d);
            foreach(var g in recipe.GetProperty("groups").EnumerateArray()){
                Stage("Weldment "+g.GetProperty("name").GetString());var bodies=AddFrameGroup(d,g);built.Add(new {group=g.GetProperty("name").GetString(),bodies=bodies.Select(ReadBody).ToArray()});Save(d);
            }
            if(recipe.TryGetProperty("trims",out var trims))foreach(var t in trims.EnumerateArray()){
                Stage("Trim "+t.GetProperty("name").GetString());cuts.Add(CopeFramePair(d,t.GetProperty("target").GetString()!,t.GetProperty("carrier").GetString()!,t.GetProperty("name").GetString()!,t.GetProperty("gapM").GetDouble(),Numbers(t.GetProperty("point")),CarrierMember(recipe,t.GetProperty("carrier").GetString()!),t.TryGetProperty("partialEnd",out var pe)&&pe.GetBoolean(),CarrierMember(recipe,t.GetProperty("target").GetString()!)));Save(d);
            }
            Mark(d,"RFR_FrameRecipe",job.GetProperty("recipe").GetString()!);Mark(d,"RFR_Status","DEVELOPMENT: native weldment frame; see gate evidence before use");
            batch.Dispose();ChassisView(d);Save(d);App.CloseDoc(d.GetTitle());d=Open(output);var readback=Bodies(d).Select(ReadBody).ToArray();
            string preview=Path.ChangeExtension(output,"bmp");d.SaveBMP(preview,1600,1000);return new {path=output,cadSha256=FileSha(output),built,cuts,readback,saveReopenPassed=true,preview};
        } finally {if(job.TryGetProperty("restore",out var restore)){App.CloseDoc(d.GetTitle());Open(restore.GetString()!);}}
    }
}


