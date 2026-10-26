using System.Runtime.InteropServices;
using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    // VR3 quote package from a finished weldment, made on a copy under cad/exports so
    // the working part keeps its semantic tube names:
    //  - cut list items created, automatic cut list turned OFF, one item per tube
    //    (left/right tubes are never merged), PARTNUMBER 001.. on each item and body
    //  - drawing: sheet 1 isometric with part-number balloons + weldment cut list;
    //    sheet 2 front/top/right views with an overall-size note; saved as SLDDRW + PDF
    //  - cut list table exported as CSV for the Excel BOM (scripts/vr3_bom.py)
    static object Vr3Package(JsonElement job) {
        string source=Safe(job.GetProperty("path").GetString()!);
        string outDir=Path.GetFullPath(job.GetProperty("outputDir").GetString()!);
        if(!outDir.StartsWith(Path.Combine(Root,"cad","exports")+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))throw new Exception("VR3 package must be written under cad/exports");
        if(Directory.Exists(outDir)&&Directory.EnumerateFileSystemEntries(outDir).Any())throw new Exception("Fresh VR3 package directory required");
        Directory.CreateDirectory(outDir);
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        string stem=Path.GetFileNameWithoutExtension(source);
        string part=Path.Combine(outDir,stem+"-VR3.SLDPRT");
        if(App.GetOpenDocumentByName(source) is ModelDoc2 live&&live.GetSaveFlag())throw new Exception("Source part has unsaved changes");
        File.Copy(source,part);
        // Build order: hoops first (they are placed first in the jig), then recipe members.
        var order=new List<string>{"MainHoop_1","FrontHoop_1"};
        order.AddRange(recipe.GetProperty("groups").EnumerateArray().SelectMany(g=>g.GetProperty("members").EnumerateArray()).Select(m=>m.GetProperty("name").GetString()!));
        var profileOf=recipe.GetProperty("groups").EnumerateArray().SelectMany(g=>g.GetProperty("members").EnumerateArray())
            .ToDictionary(m=>m.GetProperty("name").GetString()!,m=>Path.GetFileNameWithoutExtension(m.GetProperty("profile").GetString()!));
        profileOf["MainHoop_1"]=profileOf["FrontHoop_1"]="1.000 x 0.095";
        var d=Open(part);object result;
        try {
            if(Arr(((PartDoc)d).GetBodies2((int)swBodyType_e.swSheetBody,false)).Length>0)DeleteSheetBodies(d,"VR3_DeleteCutterSurfaces");
            var excluded=recipe.TryGetProperty("referenceOnlyBodies",out var r)?r.EnumerateArray().Select(x=>x.GetString()!).ToHashSet():new HashSet<string>();
            var refs=Bodies(d).Where(b=>excluded.Contains(b.Name)).ToArray();
            if(refs.Length>0) {  // keep-out/reference solids are not tubes to quote
                d.ClearSelection2(true);for(int i=0;i<refs.Length;i++)if(!refs[i].Select2(i>0,null))throw new Exception("Select reference body");
                var del=d.FeatureManager.InsertDeleteBody2(false)??throw new Exception("Remove reference solids failed");del.Name="VR3_RemoveReferenceSolids";
                d.ClearSelection2(true);d.EditRebuild3();
            }
            var tubes=Bodies(d);
            var unknown=tubes.Where(b=>!order.Contains(b.Name)).Select(b=>b.Name).ToArray();
            if(unknown.Length>0)throw new Exception("Bodies not in the recipe: "+string.Join(", ",unknown));
            var number=order.Where(n=>tubes.Any(b=>b.Name==n)).Select((n,i)=>(n,pn:(i+1).ToString("000"))).ToDictionary(x=>x.n,x=>x.pn);

            // Cut list: generate items, then freeze (VR3 checklist: automatic cut list OFF).
            var cutListFeature=Features(d).FirstOrDefault(f=>f.GetTypeName2()=="SolidBodyFolder")??throw new Exception("No weldment cut list folder");
            var root=(BodyFolder)cutListFeature.GetSpecificFeature2();
            root.SetAutomaticCutList(true);root.UpdateCutList();d.EditRebuild3();
            root.SetAutomaticCutList(false);d.EditRebuild3();
            // One item per tube (VR3 cut list step 4): SolidWorks groups left/right tubes it
            // believes identical. With automatic cut lists off, delete each such item (its
            // bodies return to the cut list root), then create one item per body.
            var loose=new List<string>();
            foreach(var item in SubFeatures(cutListFeature).Where(f=>f.GetTypeName2()=="CutListFolder").ToList()) {
                var names=Arr(((BodyFolder)item.GetSpecificFeature2()).GetBodies()).Cast<Body2>().Select(b=>b.Name).ToArray();
                if(names.Length<2)continue;
                string itemName=item.Name;d.ClearSelection2(true);
                if(!item.Select2(false,-1))throw new Exception("Select merged cut list item "+itemName);
                d.EditDelete();d.ClearSelection2(true);
                if(SubFeatures(cutListFeature).Any(f=>f.Name==itemName))throw new Exception("Delete merged cut list item failed: "+itemName);
                loose.AddRange(names);
            }
            d.EditRebuild3();
            foreach(var name in loose) {
                var body=Bodies(d).FirstOrDefault(b=>b.Name==name)??throw new Exception("Body lost after cut list item delete: "+name);
                // COM needs IDispatch-wrapped array elements; else select the body and use
                // the selection-based command ('Create Cut List Item' in the guide).
                var made=d.FeatureManager.InsertWeldmentCutList2(new DispatchWrapper[]{new DispatchWrapper(body)});
                if(made is null) {
                    d.ClearSelection2(true);
                    if(!body.Select2(false,null))throw new Exception("Select loose body "+name);
                    made=d.FeatureManager.InsertWeldmentCutList();d.ClearSelection2(true);
                }
                if(made is null)throw new Exception("Create cut list item failed for "+name);
            }
            d.EditRebuild3();
            var items=SubFeatures(cutListFeature).Where(f=>f.GetTypeName2()=="CutListFolder").ToList();
            var rows=new List<object>();
            foreach(var item in items) {
                var bodies=Arr(((BodyFolder)item.GetSpecificFeature2()).GetBodies()).Cast<Body2>().ToArray();
                if(bodies.Length!=1)throw new Exception($"Cut list item {item.Name} holds {bodies.Length} bodies after splitting");
                string tube=bodies[0].Name,pn=number[tube];
                var cpm=item.CustomPropertyManager;
                cpm.Add3("PARTNUMBER",(int)swCustomInfoType_e.swCustomInfoText,pn,(int)swCustomPropertyAddOption_e.swCustomPropertyReplaceValue);
                cpm.Add3("RFR_TUBE",(int)swCustomInfoType_e.swCustomInfoText,tube,(int)swCustomPropertyAddOption_e.swCustomPropertyReplaceValue);
                cpm.Get6("LENGTH",false,out string _,out string len,out bool _,out bool _);
                cpm.Get6("DESCRIPTION",false,out string _,out string desc,out bool _,out bool _);
                if(string.IsNullOrWhiteSpace(desc)&&profileOf.TryGetValue(tube,out var prof)) {  // e.g. 1.375 profile without a description
                    desc="Tube "+prof;cpm.Add3("DESCRIPTION",(int)swCustomInfoType_e.swCustomInfoText,desc,(int)swCustomPropertyAddOption_e.swCustomPropertyReplaceValue);
                }
                item.Name=pn+" "+tube;bodies[0].Name=pn;  // VR3: number both the item and its body
                rows.Add(new {partNumber=pn,tube,description=desc,length=len});
            }
            d.EditRebuild3();ChassisView(d);d.NameView("VR3 Isometric");Save(d);
            var (lo,hi)=Extremes(d);
            File.WriteAllText(Path.Combine(outDir,stem+"-partnumbers.json"),JsonSerializer.Serialize(rows,Json));
            // The drawing is a separate job (vr3-drawing) unless asked for here: a large frame
            // needs most of one native time budget for the cut list alone.
            object? drawing=job.TryGetProperty("drawing",out var dj)&&dj.GetBoolean()?MakeVr3Drawing(part,outDir,stem,lo,hi,rows.Count):null;
            result=new {part,partSha256=FileSha(part),tubes=rows.Count,cutList=rows,overallM=new{length=hi[0]-lo[0],width=hi[1]-lo[1],height=hi[2]-lo[2]},drawing};
        } finally {App.CloseDoc(d.GetTitle());}
        if(job.TryGetProperty("restore",out var restore)&&File.Exists(Safe(restore.GetString()!)))Open(restore.GetString()!);
        return result;
    }

    static IEnumerable<Feature> Features(ModelDoc2 d){for(var f=(Feature?)d.FirstFeature();f!=null;f=(Feature?)f.GetNextFeature())yield return f;}
    static IEnumerable<Feature> SubFeatures(Feature parent){for(var f=(Feature?)parent.GetFirstSubFeature();f!=null;f=(Feature?)f.GetNextSubFeature())yield return f;}

    // Drawing for a VR3 part already numbered by vr3-package (cad/exports/.../<stem>-VR3.SLDPRT).
    static object Vr3Drawing(JsonElement job) {
        string part=Safe(job.GetProperty("path").GetString()!);
        if(!part.StartsWith(Path.Combine(Root,"cad","exports")+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase)||!part.EndsWith("-VR3.SLDPRT",StringComparison.OrdinalIgnoreCase))
            throw new Exception("vr3-drawing takes the numbered -VR3.SLDPRT under cad/exports");
        string outDir=Path.GetDirectoryName(part)!,stem=Path.GetFileNameWithoutExtension(part)[..^4];
        foreach(var ext in new[]{"-VR3.SLDDRW","-VR3.pdf","-cutlist.csv"})if(File.Exists(Path.Combine(outDir,stem+ext)))throw new Exception("Drawing output already exists: "+stem+ext);
        var d=Open(part);object result;
        try {
            var (lo,hi)=Extremes(d);
            result=MakeVr3Drawing(part,outDir,stem,lo,hi,Bodies(d).Length,job.TryGetProperty("titleBlock",out var tb)?tb:null);
        } finally {App.CloseDoc(d.GetTitle());}
        if(job.TryGetProperty("restore",out var restore)&&File.Exists(Safe(restore.GetString()!)))Open(restore.GetString()!);
        return result;
    }

    // Title block: drawing properties the template reads ($PRP), and the format notes that read the part
    // ($PRPSHEET Description/Material) or hold the template's placeholder company name. Every sheet has its
    // own format. The part itself is not changed.
    static object ApplyTitleBlock(ModelDoc2 dd,DrawingDoc drw,JsonElement tb) {
        string G(string k)=>tb.TryGetProperty(k,out var x)?(x.GetString()??"").Replace("\n","\r\n"):"";
        var cpm=dd.Extension.CustomPropertyManager[""];
        foreach(var (prop,key) in new[]{("DrawnBy","drawnBy"),("DrawnDate","date"),("COMPANYNAME","company"),("Weight","weight"),("Revision","revision")})
            cpm.Add3(prop,(int)swCustomInfoType_e.swCustomInfoText,G(key),(int)swCustomPropertyAddOption_e.swCustomPropertyReplaceValue);
        void Height(Note n,double h){var an=(Annotation)n.GetAnnotation();var tf=(TextFormat)an.GetTextFormat(0);tf.CharHeight=h;an.SetTextFormat(0,false,tf);}
        var edited=new List<string>();
        foreach(var sheet in Arr(drw.GetSheetNames()).Select(x=>x!.ToString()!)) {
            drw.ActivateSheet(sheet);drw.EditTemplate();
            try {
                foreach(var n in Arr(((View)drw.GetFirstView()).GetNotes()).OfType<Note>()) {
                    string link=n.PropertyLinkedText??"";
                    if(link=="$PRPSHEET:\"Description\""){n.SetText(G("title"));Height(n,.004);edited.Add(sheet+": title");}
                    else if(link=="$PRPSHEET:\"Material\""){n.SetText(G("material"));edited.Add(sheet+": material");}
                    else if(link=="$PRP:\"SW-File Name\""){Height(n,.003);edited.Add(sheet+": drawing number height");}
                    else if(link=="$PRP:\"COMPANYNAME\""){Height(n,.003);edited.Add(sheet+": company name height");}
                    else if(link.Contains("<INSERT COMPANY NAME HERE>")){n.SetText(link.Replace("<INSERT COMPANY NAME HERE>",G("company")));edited.Add(sheet+": proprietary note");}
                }
            } finally {drw.EditSheet();}
        }
        return new {properties=new{drawnBy=G("drawnBy"),date=G("date"),company=G("company"),weight=G("weight"),revision=G("revision")},title=G("title"),material=G("material"),edited};
    }

    // Exact overall extents of the frame (GetBodyBox can be loose on curved faces).
    static (double[] lo,double[] hi) Extremes(ModelDoc2 d) {
        var bodies=Bodies(d);double[] lo=new double[3],hi=new double[3];
        for(int k=0;k<3;k++)foreach(var sign in new[]{-1.0,1.0}) {
            double best=double.NegativeInfinity;
            foreach(var b in bodies){var u=new double[3];u[k]=sign;b.GetExtremePoint(u[0],u[1],u[2],out double x,out double y,out double z);best=Math.Max(best,sign*new[]{x,y,z}[k]);}
            if(sign<0)lo[k]=-best;else hi[k]=best;
        }
        return (lo,hi);
    }

    static object MakeVr3Drawing(string part,string outDir,string stem,double[] lo,double[] hi,int tubes,JsonElement? titleBlock=null) {
        // Project copy of the installed ANSI (inch) template first, then the SolidWorks default.
        string template=Path.Combine(Root,"cad","templates","RFR-drawing-ansi.drwdot");
        if(!File.Exists(template))template=App.GetUserPreferenceStringValue((int)swUserPreferenceStringValue_e.swDefaultTemplateDrawing);
        if(string.IsNullOrEmpty(template)||!File.Exists(template))throw new Exception("No drawing template found: "+template);
        // Sheet size by cut list length: the table runs down the left edge (C <= 40 rows, D <= 60, else E).
        var (paper,tpl,W,H)=tubes<=40?((int)swDwgPaperSizes_e.swDwgPaperCsize,(int)swDwgTemplates_e.swDwgTemplateCsize,.5588,.4318)
                            :tubes<=60?((int)swDwgPaperSizes_e.swDwgPaperDsize,(int)swDwgTemplates_e.swDwgTemplateDsize,.8636,.5588)
                            :((int)swDwgPaperSizes_e.swDwgPaperEsize,(int)swDwgTemplates_e.swDwgTemplateEsize,1.1176,.8636);
        var dd=App.NewDocument(template,paper,0,0) as ModelDoc2??throw new Exception("Drawing creation failed");
        try {
        var drw=(DrawingDoc)dd;
        string tableTemplate=Path.Combine(App.GetExecutablePath(),"lang","english","cut list.sldwldtbt");
        string sheet1=((Sheet)drw.GetCurrentSheet()).GetName();
        drw.SetupSheet5(sheet1,paper,tpl,1,1,false,"",0,0,"",true);
        // Model sketches, planes and axes stay out of the views (they showed as stray lines at the bulkheads).
        dd.Extension.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swViewDisplayHideAllTypes,(int)swUserPreferenceOption_e.swDetailingNoOptionSpecified,true);
        // Sheet 1: Z-up isometric, balloons, weldment cut list (left edge) with a PART NO. column.
        var iso=drw.CreateDrawViewFromModelView3(part,"VR3 Isometric",W*.64,H*.55,0)??throw new Exception("Isometric view failed");
        FitView(iso,W*.56,H*.70);iso.Position=new double[]{W*.64,H*.57};
        var table=iso.InsertWeldmentTable(true,W*.012,H*.985,(int)swBOMConfigurationAnchorType_e.swBOMConfigurationAnchor_TopLeft,"",tableTemplate)??throw new Exception("Weldment cut list table failed");
        var ta=(TableAnnotation)table;
        var tableProps=Arr(table.GetAllCustomProperties()).Select(x=>x?.ToString()??"").ToArray();
        // A column inserted into the table is not a property column: SetColumnCustomProperty alone
        // returned false, and the old fallback wrote item numbers as part numbers (009c package,
        // 12 Oct: all 118 rows wrong). Make it a custom-property column, then link it.
        bool Link(int col,string prop) {
            int type=(int)swTableColumnTypes_e.swWeldTableColumnType_CustomProperty;
            ta.SetColumnType3(col,type,false,prop);
            if(table.GetColumnCustomProperty(col)!=prop){ta.SetColumnType2(col,type,false);table.SetColumnCustomProperty(col,prop);}
            return table.GetColumnCustomProperty(col)==prop;
        }
        int width=(int)swInsertTableColumnWidthStyle_e.swInsertColumn_DefaultWidth,after=(int)swTableItemInsertPosition_e.swTableItemInsertPosition_After;
        if(!ta.InsertColumn2(after,0,"PART NO.",width))throw new Exception("Part number column failed");
        var expected=JsonDocument.Parse(File.ReadAllText(Path.Combine(outDir,stem+"-partnumbers.json"))).RootElement.EnumerateArray()
            .ToDictionary(x=>x.GetProperty("partNumber").GetString()!,x=>(tube:x.GetProperty("tube").GetString()!,length:x.GetProperty("length").GetString()!,desc:x.GetProperty("description").GetString()!));
        // Balloons go on before the table is linked and sorted: placed afterwards, AutoBalloon's square
        // layout ran off the sheet (009c r2 attempts, 12 Oct). Balloons show item numbers and follow the sort.
        dd.ClearSelection2(true);if(!dd.Extension.SelectByID2(iso.GetName2(),"DRAWINGVIEW",0,0,0,false,0,null,0))throw new Exception("Select isometric view");
        var opts=drw.CreateAutoBalloonOptions();opts.Layout=(int)swBalloonLayoutType_e.swDetailingBalloonLayout_Square;opts.IgnoreMultiple=true;
        opts.LeaderAttachmentToFaces=false;  // face attachment placed only 94 of 118 balloons (009c, 12 Oct); edges place all
        opts.Style=(int)swBalloonStyle_e.swBS_Circular;opts.Size=(int)swBalloonFit_e.swBF_3Chars;
        opts.UpperTextContent=(int)swBalloonTextContent_e.swBalloonTextItemNumber;
        var balloons=Arr(drw.AutoBalloon5(opts));dd.ClearSelection2(true);
        if(balloons.Length!=expected.Count)throw new Exception($"{balloons.Length} balloons for {expected.Count} cut list items");
        if(!Link(1,"PARTNUMBER"))throw new Exception("PART NO. column could not be linked to the PARTNUMBER cut-list property");
        ta.SetColumnTitle2(1,"PART NO.",false);
        bool sorted=table.Sort(1,true);
        // Check every row against the part's own numbering (vr3-package wrote <stem>-partnumbers.json
        // from each cut-list item's properties), through a temporary column linked to RFR_TUBE.
        // Long tube names wrap and grow the rows; restore the row heights after the temporary column.
        var heights=Enumerable.Range(0,ta.RowCount).Select(r=>ta.GetRowHeight(r)).ToArray();
        if(!ta.InsertColumn2(after,1,"RFR_TUBE",width)||!Link(2,"RFR_TUBE"))throw new Exception("Temporary tube column failed");
        int Col(string title){for(int c=0;c<ta.ColumnCount;c++)if((ta.GetColumnTitle2(c,false)??"").Trim().Equals(title,StringComparison.OrdinalIgnoreCase))return c;throw new Exception("Cut list column missing: "+title);}
        int iLen=Col("LENGTH"),iDesc=Col("DESCRIPTION");
        var bad=new List<string>();var seen=new HashSet<string>();bool itemIsPart=true;
        for(int r=1;r<ta.RowCount;r++) {
            string item=ta.get_Text(r,0).Trim(),pn=ta.get_Text(r,1).Trim(),tube=ta.get_Text(r,2).Trim(),len=ta.get_Text(r,iLen).Trim(),desc=ta.get_Text(r,iDesc).Trim();
            if(!expected.TryGetValue(pn,out var x)||x.tube!=tube||x.length!=len||x.desc!=desc||!seen.Add(pn))bad.Add($"row {r}: {pn} {tube} {len} {desc}");
            if(!int.TryParse(item,out int n)||n.ToString("000")!=pn)itemIsPart=false;
        }
        if(seen.Count!=expected.Count)bad.Add($"{seen.Count} of {expected.Count} part numbers in the table");
        if(bad.Count>0)throw new Exception("Cut list table differs from the part's numbering: "+string.Join("; ",bad.Take(8)));
        if(!ta.DeleteColumn2(2,false))throw new Exception("Temporary tube column could not be removed");
        // Data rows a little shorter than before so the last rows clear the Educational watermark at the sheet foot.
        for(int r=0;r<heights.Length;r++)ta.SetRowHeight(r,r==0?heights[r]:heights[r]*.94,(int)swTableRowColSizeChangeBehavior_e.swTableRowColChange_TableSizeCanChange);
        // Balloons show item numbers (a cut-list-property balloon came out blank). After the sort the
        // item number of every row equals its part number, checked above against the part's numbering.
        if(!itemIsPart)throw new Exception("Cut list item numbers do not equal part numbers after sorting; balloons would not match");
        // Each balloon must show its tube's part number: vr3-package names every body by its part number.
        var balloonBad=new List<string>();int balloonChecked=0,balloonUnresolved=0;
        foreach(var b in balloons) {
            if(b is not Note n){balloonUnresolved++;continue;}
            string text=(n.GetText()??"").Trim();
            string? body=Arr(((Annotation)n.GetAnnotation()).GetAttachedEntities3()).Select(x=>x switch{Edge ed=>(ed.GetBody() as Body2)?.Name,Face2 f=>(f.GetBody() as Body2)?.Name,_=>null}).FirstOrDefault(x=>x!=null);
            if(body==null||text.Length==0){balloonUnresolved++;continue;}
            if(int.TryParse(text,out int k)&&k.ToString("000")==body)balloonChecked++;else balloonBad.Add($"{text}->{body}");
        }
        if(balloonBad.Count>0)throw new Exception("Balloons differ from the part numbers of their tubes: "+string.Join(", ",balloonBad.Take(8)));
        string csv=Path.Combine(outDir,stem+"-cutlist.csv");
        if(!ta.SaveAsText2(csv,",",false))throw new Exception("Cut list CSV export failed");
        // Sheet 2: top, front and side views of the Z-up / X-forward frame at one scale. In the team part
        // *Top looks down z, *Front along x and *Right along y (named views checked by orientation, 12 Oct).
        if(!drw.NewSheet4("Views",paper,tpl,1,1,false,"",0,0,"",0,0,0,0,0,0))throw new Exception("Sheet 2 failed");
        var side=drw.CreateDrawViewFromModelView3(part,"*Top",W*.30,H*.72,0)??throw new Exception("Top view failed");
        var plan=drw.CreateDrawViewFromModelView3(part,"*Front",W*.30,H*.38,0)??throw new Exception("Front view failed");
        var front=drw.CreateDrawViewFromModelView3(part,"*Right",W*.78,H*.72,0)??throw new Exception("Side view failed");
        double fit=new[]{side,plan,front}.Min(v=>{var o=(double[])v.GetOutline();return v.ScaleDecimal*Math.Min(W*.5/(o[2]-o[0]),H*.3/(o[3]-o[1]));});
        // One standard scale for the sheet and its views, so the title block's SCALE is true for sheet 2.
        double den=new[]{1,1.5,2,2.5,3,4,5,6,8,10,12,15,16,20,24,25,30,40,50,60,80,100}.First(q=>1/q<=fit),scale=1/den;
        if(!((Sheet)drw.GetCurrentSheet()).SetScale(1,den,false,false))throw new Exception("Sheet 2 scale 1:"+den+" failed");
        foreach(var v in new[]{side,plan,front})v.ScaleDecimal=scale;
        side.Position=new double[]{W*.30,H*.72};plan.Position=new double[]{W*.30,H*.38};front.Position=new double[]{W*.78,H*.72};
        string size=$"OVERALL (in): LENGTH {(hi[0]-lo[0])/.0254:0.00}  WIDTH {(hi[1]-lo[1])/.0254:0.00}  HEIGHT {(hi[2]-lo[2])/.0254:0.00}";
        // (The per-view notes below carry the overall extents; size is kept in the job result.)
        // Each view is labelled by its line of sight, with the frame's exact overall extents in that view
        // (VR3 checklist: relevant lengths, widths and heights). These are notes, not dimensions: API sketch
        // points land on the sheet and dimension at paper size, coordinate picks at the outline select
        // nothing, and an entity search of each view ran past the native time budget (12 Oct). Add true
        // dimensions by hand in the SOLIDWORKS UI if the reviewer asks; the notes give the values to match.
        var mu=(MathUtility)App.GetMathUtility();var viewDims=new List<object>();string[] axisName={"LENGTH","WIDTH","HEIGHT"};
        foreach(var v in new[]{side,plan,front}) {
            var t=v.ModelToViewTransform;
            double[] Sheet(double x,double y,double z)=>(double[])((MathPoint)((MathPoint)mu.CreatePoint(new[]{x,y,z})).MultiplyTransform(t)).ArrayData;
            double[] Dir(int k){var u=new double[3];u[k]=1;return (double[])((MathVector)((MathVector)mu.CreateVector(u)).MultiplyTransform(t)).ArrayData;}
            var dirs=Enumerable.Range(0,3).Select(Dir).ToArray();
            int depth=Enumerable.Range(0,3).OrderByDescending(k=>Math.Abs(dirs[k][2])).First();
            int hAxis=Enumerable.Range(0,3).Where(k=>k!=depth).OrderByDescending(k=>Math.Abs(dirs[k][0])).First(),vAxis=3-depth-hAxis;
            var corners=(from x in new[]{lo[0],hi[0]} from y in new[]{lo[1],hi[1]} from z in new[]{lo[2],hi[2]} select Sheet(x,y,z)).ToArray();
            double x0=corners.Min(c=>c[0]),x1=corners.Max(c=>c[0]),y0=corners.Min(c=>c[1]),y1=corners.Max(c=>c[1]);
            string label=depth==2?"TOP VIEW":depth==1?"SIDE VIEW":"FRONT VIEW";
            string text=$"{label}{System.Environment.NewLine}OVERALL {axisName[hAxis]} {(hi[hAxis]-lo[hAxis])/.0254:0.00} in, {axisName[vAxis]} {(hi[vAxis]-lo[vAxis])/.0254:0.00} in";
            var lab=dd.InsertNote(text) as Note??throw new Exception("View label failed: "+label);((Annotation)lab.GetAnnotation()).SetPosition2(x0,y0-.012,0);
            viewDims.Add(new {view=v.GetName2(),label,note=text,sheetBox=new[]{x0,y0,x1,y1}});
        }
        drw.ActivateSheet("Views");
        object? titleBlockApplied=titleBlock is JsonElement tbe?ApplyTitleBlock(dd,drw,tbe):null;
        drw.ActivateSheet(sheet1);
        string dwg=Path.Combine(outDir,stem+"-VR3.SLDDRW"),pdf=Path.Combine(outDir,stem+"-VR3.pdf");int e=0,w=0;
        if(!dd.Extension.SaveAs(dwg,0,1,null,ref e,ref w)||e!=0)throw new Exception($"Drawing save failed {e},{w}");
        var pdfData=(ExportPdfData)App.GetExportFileData((int)swExportDataFileType_e.swExportPdfData);
        pdfData.SetSheets((int)swExportDataSheetsToExport_e.swExportData_ExportAllSheets,null);
        if(!dd.Extension.SaveAs(pdf,0,1,pdfData,ref e,ref w)||e!=0)throw new Exception($"PDF export failed {e},{w}");
        return new {template,drawing=dwg,pdf,csv,balloons=balloons.Length,balloonText="item number (== part number)",balloonChecked,balloonUnresolved,sheets=2,overallNote=size,partNumberColumnLinked=true,sortedByPartNumber=sorted,itemNumberEqualsPartNumber=itemIsPart,
            rowsCheckedAgainstPartNumbers=seen.Count,sheet2Scale="1:"+den,viewDims,titleBlockApplied,overallM=new{length=hi[0]-lo[0],width=hi[1]-lo[1],height=hi[2]-lo[2]},tableProperties=tableProps};
        } finally {App.CloseDoc(dd.GetTitle());}
    }

    static void FitView(View v,double maxW,double maxH) {
        var o=(double[])v.GetOutline();double w=o[2]-o[0],h=o[3]-o[1];
        double s=v.ScaleDecimal*Math.Min(maxW/w,maxH/h);v.ScaleDecimal=s;
    }
}
