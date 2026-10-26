using System.Text.Json;
using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    static double[] Sub(double[] a,double[] b)=>a.Zip(b,(x,y)=>x-y).ToArray();
    static double Dot(double[] a,double[] b)=>a.Zip(b,(x,y)=>x*y).Sum();
    static double Norm(double[] a)=>Math.Sqrt(Dot(a,a));
    static double[] Unit(double[] a)=>a.Select(v=>v/Norm(a)).ToArray();
    static double[] Cross(double[] a,double[] b)=>new[]{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
    static object AuditMainHoop(ModelDoc2 d,string recipePath) {
        var recipe=JsonDocument.Parse(File.ReadAllText(recipePath)).RootElement;
        var pathFeature=Find(d,"MainHoop_TangentPath");var member=Find(d,"MainHoop_Repaired");
        if(pathFeature==null||member==null)return new {accepted=false,reason="Expected native repaired path/member not present"};
        var sk=(Sketch)pathFeature.GetSpecificFeature2();
        var native=Arr(sk.GetSketchSegments()).Cast<SketchSegment>().Where(s=>!s.ConstructionGeometry).ToList();
        var mu=(MathUtility)App.GetMathUtility();var inverse=sk.ModelToSketchTransform.Inverse();
        double[] World(SketchPoint p)=>(double[])((MathPoint)((MathPoint)mu.CreatePoint(new[]{p.X,p.Y,p.Z})).MultiplyTransform(inverse)).ArrayData;
        (double[] a,double[] b) Ends(SketchSegment s)=>s.GetType()==0?
            (World((SketchPoint)((SketchLine)s).GetStartPoint2()),World((SketchPoint)((SketchLine)s).GetEndPoint2())):
            (World((SketchPoint)((SketchArc)s).GetStartPoint2()),World((SketchPoint)((SketchArc)s).GetEndPoint2()));
        var measures=new List<object>();var starts=new List<double[]>();var ends=new List<double[]>();
        var startTangents=new List<double[]>();var endTangents=new List<double[]>();
        double length=0,maxMatch=0;bool radiiMatch=true;
        foreach(var s in recipe.GetProperty("segments").EnumerateArray()) {
            double[] a=Numbers(s.GetProperty("start")),b=Numbers(s.GetProperty("end"));int kind=s.GetProperty("kind").GetString()=="line"?0:1;
            var match=native.Where(x=>x.GetType()==kind).Select(x=>{var e=Ends(x);double forward=Norm(Sub(e.a,a))+Norm(Sub(e.b,b)),reverse=Norm(Sub(e.b,a))+Norm(Sub(e.a,b));return new {x,e,reversed=reverse<forward,error=Math.Min(forward,reverse)};}).OrderBy(x=>x.error).FirstOrDefault();
            if(match==null||match.error>1e-7)return new {accepted=false,reason="Native centerline differs from repair recipe"};
            native.Remove(match.x);maxMatch=Math.Max(maxMatch,match.error);
            a=match.reversed?match.e.b:match.e.a;b=match.reversed?match.e.a:match.e.b;starts.Add(a);ends.Add(b);
            if(kind==0){var v=Sub(b,a);double l=Norm(v);length+=l;startTangents.Add(Unit(v));endTangents.Add(Unit(v));measures.Add(new {kind="line",lengthM=l});}
            else {
                var arc=(SketchArc)match.x;var c=World((SketchPoint)arc.GetCenterPoint2());double radius=arc.GetRadius();
                var ra=Sub(a,c);var rb=Sub(b,c);var n=Unit(Cross(ra,rb));double angle=Math.Acos(Math.Clamp(Dot(Unit(ra),Unit(rb)),-1,1));
                length+=radius*angle;startTangents.Add(Unit(Cross(n,ra)));endTangents.Add(Unit(Cross(n,rb)));
                radiiMatch &= Math.Abs(radius-s.GetProperty("radiusM").GetDouble())<1e-8&&Norm(Sub(c,Numbers(s.GetProperty("center"))))<1e-8;
                measures.Add(new {kind="arc",radiusM=radius,angleDeg=angle*180/Math.PI});
            }
        }
        double maxGap=0,maxJump=0;
        for(int i=1;i<starts.Count;i++){maxGap=Math.Max(maxGap,Norm(Sub(starts[i],ends[i-1])));maxJump=Math.Max(maxJump,Math.Acos(Math.Clamp(Dot(startTangents[i],endTangents[i-1]),-1,1))*180/Math.PI);}
        var hoop=Bodies(d).Where(b=>b.Name.StartsWith("MainHoop_")).ToArray();
        double od=.0254,wall=.095*.0254,area=Math.PI/4*(od*od-Math.Pow(od-2*wall,2));
        double expected=area*length,actual=hoop.Sum(Volume),relativeError=Math.Abs(actual-expected)/expected;
        bool accepted=hoop.Length==1&&hoop[0].Check3?.Count==0&&member.GetErrorCode()==0&&native.Count==0&&radiiMatch&&maxGap<1e-8&&maxJump<.001&&relativeError<1e-5;
        return new {accepted,recipeSha256=FileSha(recipePath),nativeSegmentCount=starts.Count,bodyCount=hoop.Length,maxEndpointMismatchM=maxMatch,
            maxJoinGapM=maxGap,maxTangentJumpDeg=maxJump,radiiMatch,lengthM=length,expectedUncutTubeVolumeM3=expected,nativeVolumeM3=actual,relativeVolumeError=relativeError,segments=measures,
            scope="One native uncut tube matching the tangent bend recipe; full frame, fit and manufacturing access are separate gates"};
    }
}
