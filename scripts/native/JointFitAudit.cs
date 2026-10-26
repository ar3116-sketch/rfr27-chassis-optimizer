using System.Text.Json;
using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    sealed record AuditFace(Face2 face,double[] box,string body);
    static double BoxPointDistance(double[] b,double[] p)=>Math.Sqrt(Enumerable.Range(0,3).Sum(i=>Math.Pow(Math.Max(0,Math.Max(b[i]-p[i],p[i]-b[i+3])),2)));
    static object JointFitAudit(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        var refs=recipe.GetProperty("referenceOnlyBodies").EnumerateArray().Select(x=>x.GetString()!).ToHashSet();
        var tubes=Bodies(d).Where(b=>!refs.Contains(b.Name)).ToArray();
        var bodyBounds=tubes.ToDictionary(b=>b.Name,b=>(double[])b.GetBodyBox());
        var faceSets=tubes.ToDictionary(b=>b.Name,b=>Arr(b.GetFaces()).Cast<Face2>().Select(f=>new AuditFace(f,(double[])f.GetBox(),b.Name)).ToArray());
        var reports=new List<object>();bool all=true;
        try {
            foreach(var m in recipe.GetProperty("members").EnumerateArray()) {
                string name=m.GetProperty("name").GetString()!;Stage("Checking complete end boundaries "+name);
                var a=Numbers(m.GetProperty("a"));var b=Numbers(m.GetProperty("b"));var axis=Unit(Sub(b,a));
                var body=tubes.Single(t=>t.Name==name);
                double wall=m.GetProperty("size").GetString() switch {"A"=>.095*.0254,"B"=>.065*.0254,"C"=>.049*.0254,_=>throw new Exception("Unsupported stock")};
                var radii=faceSets[name].Select(f=>(Surface)f.face.GetSurface()).Where(s=>s.IsCylinder()).Select(s=>(double[])s.CylinderParams)
                    .Where(c=>Math.Abs(Math.Abs(Dot(c.Skip(3).Take(3).ToArray(),axis))-1)<1e-7&&Norm(Cross(Sub(c.Take(3).ToArray(),a),axis))<1e-7).Select(c=>c[6]).ToArray();
                bool sectionValid=radii.Length>=2&&radii.All(q=>Math.Abs(q-.0127)<1e-7||Math.Abs(q-(.0127-wall))<1e-7)&&radii.Any(q=>Math.Abs(q-.0127)<1e-7)&&radii.Any(q=>Math.Abs(q-(.0127-wall))<1e-7);
                var cutFaces=faceSets[name].Where(f=>{
                    var surface=(Surface)f.face.GetSurface();if(!surface.IsCylinder())return true;
                    var c=(double[])surface.CylinderParams;
                    return Math.Abs(Math.Abs(Dot(c.Skip(3).Take(3).ToArray(),axis))-1)>1e-7||Norm(Cross(Sub(c.Take(3).ToArray(),a),axis))>1e-7;
                }).ToArray();
                var edges=cutFaces.SelectMany(f=>Arr(f.face.GetEdges()).Cast<Edge>()).ToArray();
                var report=new List<object>();double maxSample=0,maxCertified=0;int samples=0;
                foreach(var edge in edges) {
                    var curve=(Curve)edge.GetCurve();var cp=(CurveParamData)edge.GetCurveParams3();double start=cp.UMinValue,end=cp.UMaxValue;
                    double length=curve.GetLength3(start,end);int count=Math.Max(8,(int)Math.Ceiling(length/.001));
                    if(count>2048||length<=0||!double.IsFinite(length))throw new Exception("Unsupported cut edge length");
                    double[] previousPoint=Array.Empty<double>();double previousGap=0,previousT=start;double edgeBound=0;
                    for(int i=0;i<=count;i++) {
                        double t=start+(end-start)*i/count;var p=((double[])curve.Evaluate2(t,0)).Take(3).ToArray();double gap=double.PositiveInfinity;
                        var near=faceSets.Where(v=>v.Key!=name&&BoxPointDistance(bodyBounds[v.Key],p)<.020).SelectMany(v=>v.Value).Select(f=>new {f,lower=BoxPointDistance(f.box,p)}).OrderBy(v=>v.lower);
                        foreach(var item in near) {
                            if(item.lower>gap+1e-8)break;
                            var closest=((double[])item.f.face.GetClosestPointOn(p[0],p[1],p[2])).Take(3).ToArray();gap=Math.Min(gap,Norm(Sub(p,closest)));
                            if(gap<1e-9)break;
                        }
                        if(!double.IsFinite(gap))gap=.020;
                        maxSample=Math.Max(maxSample,gap);samples++;
                        // Distance to the union of other surfaces is 1-Lipschitz.
                        // Native curve arc length bounds every unsampled point.
                        if(i>0)edgeBound=Math.Max(edgeBound,Math.Max(gap,previousGap)+curve.GetLength3(previousT,t)/2);
                        previousPoint=p;previousGap=gap;previousT=t;
                    }
                    maxCertified=Math.Max(maxCertified,edgeBound);report.Add(new {lengthM=length,samples=count+1,certifiedMaximumGapM=edgeBound});
                }
                bool passed=sectionValid&&edges.Length>0&&maxCertified<=.00127;all &= passed;
                reports.Add(new {name,accepted=passed,sectionValid,actualLongitudinalRadiiM=radii,cutFaceCount=cutFaces.Length,edgeCount=edges.Length,samples,maximumSampleGapM=maxSample,certifiedMaximumGapM=maxCertified,edges=report});
            }
            return new {path=d.GetPathName(),cadSha256=FileSha(d.GetPathName()),allStraightMemberEndsFit=all,maximumAllowedGapM=.00127,reports,
                method="Every boundary of every non-longitudinal tube end face sampled with native arc-length Lipschitz coverage; closest point on actual other tube faces",
                limitation="Geometric cut fit; torch access and assembly sequence are separate checks. Uncut hoop base openings allowed by F.5.3.3."};
        } finally {if(job.TryGetProperty("restore",out var restore))Open(restore.GetString()!);}
    }
}

