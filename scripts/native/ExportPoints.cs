using System.Text.Json;
using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    // Surface sample points of assembly components (e.g. the posed Percy segments)
    // in assembly coordinates, so the layout optimizer can use them as keep-outs.
    // Read-only: never saves the assembly.
    static object ExportComponentPoints(JsonElement job) {
        var asm=Open(job.GetProperty("path").GetString()!);if(asm.GetType()!=2)throw new Exception("Expected an assembly");
        string match=job.GetProperty("match").GetString()!;
        string output=Path.GetFullPath(job.GetProperty("output").GetString()!);
        if(!output.StartsWith(Path.GetFullPath(Root).TrimEnd(Path.DirectorySeparatorChar)+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))throw new Exception("Output must be inside the project");
        ((AssemblyDoc)asm).ResolveAllLightWeightComponents(false);
        var mu=(MathUtility)App.GetMathUtility();var parts=new List<object>();
        foreach(var c in Arr(((AssemblyDoc)asm).GetComponents(false)).Cast<Component2>()) {
            if(c.IsSuppressed()||c.GetModelDoc2() is not ModelDoc2 pd||pd.GetType()!=1)continue;
            string id=c.Name2+"|"+c.GetPathName();if(!id.Contains(match,StringComparison.OrdinalIgnoreCase))continue;
            var tf=c.Transform2;var pts=new List<double[]>();
            foreach(var b in Bodies(pd))foreach(var f in Arr(b.GetFaces()).Cast<Face2>()) {
                var tri=f.GetTessTriangles(true) as float[];
                if(tri==null){foreach(var v in Arr(b.GetVertices()).Cast<Vertex>())pts.Add((double[])v.GetPoint());continue;}
                for(int i=0;i+2<tri.Length;i+=3)pts.Add(new double[]{tri[i],tri[i+1],tri[i+2]});
            }
            var world=pts.Select(p=>{var mp=(MathPoint)mu.CreatePoint(p);if(tf!=null)mp=(MathPoint)mp.MultiplyTransform(tf);return ((double[])mp.ArrayData).Take(3).Select(x=>Math.Round(x,5)).ToArray();})
                .GroupBy(p=>string.Join(",",p)).Select(g=>g.First()).ToArray();
            parts.Add(new {component=c.Name2,path=c.GetPathName(),pointCount=world.Length,pointsM=world});
        }
        if(parts.Count==0)throw new Exception("No components matched "+match);
        File.WriteAllText(output,JsonSerializer.Serialize(new {assembly=asm.GetPathName(),match,exportedUtc=DateTime.UtcNow,units="m, assembly coordinates",parts},Json));
        return new {output,components=parts.Count};
    }
}
