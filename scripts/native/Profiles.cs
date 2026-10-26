using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

internal static partial class Program
{
    // VR3 weldment profile library (VR3 guide: one profile per tube size, file name and
    // description "<OD> x <wall>"). New round sizes are made from an existing round profile
    // by setting its sketch dimensions; the template file itself is never saved.
    static object MakeProfiles(JsonElement job) {
        string template=Safe(job.GetProperty("template").GetString()!);
        double tOd=job.GetProperty("templateOdIn").GetDouble()*.0254,tWall=job.GetProperty("templateWallIn").GetDouble()*.0254;
        var made=new List<object>();
        foreach(var spec in job.GetProperty("sizes").EnumerateArray()) {
            double odIn=spec.GetProperty("od").GetDouble(),wallIn=spec.GetProperty("wall").GetDouble();
            string outDir=Safe(job.GetProperty("outputDir").GetString()!+Path.DirectorySeparatorChar+"x.sldlfp");outDir=Path.GetDirectoryName(outDir)!;
            string name=$"{odIn:0.000} x {wallIn:0.000}",output=Path.Combine(outDir,name+".sldlfp");
            if(File.Exists(output)){made.Add(new{name,skipped="exists"});continue;}
            double od=odIn*.0254,wall=wallIn*.0254;
            var d=Open(template);
            try {
                var dims=new List<Dimension>();
                for(var f=(Feature?)d.FirstFeature();f!=null;f=(Feature?)f.GetNextFeature())
                    for(var dd=(DisplayDimension?)f.GetFirstDisplayDimension();dd!=null;dd=(DisplayDimension?)f.GetNextDisplayDimension(dd))
                        dims.Add((Dimension)dd.GetDimension2(0));
                var found=dims.GroupBy(x=>x.FullName).Select(g=>g.First()).Select(x=>new{x,v=x.SystemValue,name=x.FullName}).ToArray();
                // Template dimensioning: outer diameter plus inner diameter, or plus wall thickness.
                var outer=found.Where(x=>Math.Abs(x.v-tOd)<1e-6).ToArray();
                var inner=found.Where(x=>Math.Abs(x.v-(tOd-2*tWall))<1e-6).ToArray();
                var wallDim=found.Where(x=>Math.Abs(x.v-tWall)<1e-6).ToArray();
                if(outer.Length!=1||inner.Length+wallDim.Length!=1)
                    throw new Exception("Unrecognised profile dimensioning: "+string.Join("; ",found.Select(x=>$"{x.name}={x.v}")));
                int cfg=(int)swSetValueInConfiguration_e.swSetValue_InAllConfigurations;
                if(outer[0].x.SetSystemValue3(od,cfg,null)!=0)throw new Exception("Set outer diameter");
                if(inner.Length==1){if(inner[0].x.SetSystemValue3(od-2*wall,cfg,null)!=0)throw new Exception("Set inner diameter");}
                else if(wallDim[0].x.SetSystemValue3(wall,cfg,null)!=0)throw new Exception("Set wall");
                if(!d.EditRebuild3())throw new Exception("Profile rebuild failed");
                if(Math.Abs(outer[0].x.SystemValue-od)>1e-9)throw new Exception("Outer diameter readback");
                // Description and any property naming the template size follow the new size.
                string tName=$"{tOd/.0254:0.000} x {tWall/.0254:0.000}";
                var props=new List<object>();
                foreach(var cpm in new[]{d.Extension.CustomPropertyManager[""],d.Extension.CustomPropertyManager[d.ConfigurationManager.ActiveConfiguration.Name]}) {
                    foreach(var p in Arr(cpm.GetNames()).Select(x=>x?.ToString()??"")) {
                        cpm.Get6(p,false,out string raw,out string val,out bool _,out bool _);
                        if(raw.Contains(tName)){cpm.Set2(p,raw.Replace(tName,name));props.Add(new{p,from=raw,to=raw.Replace(tName,name)});}
                    }
                }
                var top=d.Extension.CustomPropertyManager[""];
                top.Get6("Description",false,out string desc,out string _,out bool _,out bool _);
                if(string.IsNullOrWhiteSpace(desc)){top.Add3("Description",(int)swCustomInfoType_e.swCustomInfoText,"Tube "+name,(int)swCustomPropertyAddOption_e.swCustomPropertyReplaceValue);props.Add(new{p="Description",to="Tube "+name});}
                int e=0,w=0;
                if(!d.Extension.SaveAs(output,(int)swSaveAsVersion_e.swSaveAsCurrentVersion,(int)(swSaveAsOptions_e.swSaveAsOptions_Silent|swSaveAsOptions_e.swSaveAsOptions_Copy),null,ref e,ref w)||e!=0||!File.Exists(output))
                    throw new Exception($"Save profile failed {e},{w}");
                made.Add(new{name,output,dimensions=found.Select(x=>new{x.name,valueM=x.v}).ToArray(),odM=od,wallM=wall,props});
            } finally {App.CloseDoc(d.GetTitle());}
        }
        return new{template,made};
    }
}
