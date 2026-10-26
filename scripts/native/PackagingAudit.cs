using System.Text.Json;
using System.Runtime.InteropServices;
using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    static Body2 PolygonPrism(double[][] points,double[] normal,double depth) {
        var modeler=(Modeler)App.GetModeler();var mu=(MathUtility)App.GetMathUtility();
        var surface=(Surface)modeler.CreatePlanarSurface2(points[0],normal,Unit(Sub(points[1],points[0])));
        var curves=new List<Curve>();
        for(int i=0;i<points.Length;i++) {
            var a=points[i];var b=points[(i+1)%points.Length];var line=(Curve)modeler.CreateLine(a,Sub(b,a));
            curves.Add(line.CreateTrimmedCurve2(a[0],a[1],a[2],b[0],b[1],b[2]));
        }
        var sheet=surface.CreateTrimmedSheet4(curves.Select(c=>new DispatchWrapper(c)).ToArray(),true) as Body2??throw new Exception("Template sheet creation failed");
        var body=modeler.CreateExtrudedBody(sheet,(MathVector)mu.CreateVector(normal),depth)??throw new Exception("Template extrusion failed");
        double area=Norm(Enumerable.Range(0,points.Length).Select(i=>Cross(points[i],points[(i+1)%points.Length])).Aggregate(new double[3],Add))/2;
        if(body.Check3?.Count!=0||Math.Abs(Volume(body)-area*depth)>Math.Max(1e-12,area*depth*1e-7))throw new Exception("Template volume/validity readback failed");
        return body;
    }
    static object PackagingAudit(JsonElement job) {
        var d=Open(job.GetProperty("path").GetString()!);
        var layout=JsonDocument.Parse(File.ReadAllText(job.GetProperty("layoutRecipe").GetString()!)).RootElement;
        var recipe=JsonDocument.Parse(File.ReadAllText(job.GetProperty("recipe").GetString()!)).RootElement;
        var excluded=recipe.GetProperty("referenceOnlyBodies").EnumerateArray().Select(x=>x.GetString()!).ToHashSet();
        var tubes=Bodies(d).Where(b=>!excluded.Contains(b.Name)).ToArray();var reports=new List<object>();
        var bodies=new List<Body2>();
        try {
            foreach(var shape in layout.GetProperty("sweptEnvelopes").EnumerateArray()) {
                string name=shape.GetProperty("name").GetString()!;Stage("Checking full swept envelope "+name);
                var body=PolygonPrism(shape.GetProperty("polygon").EnumerateArray().Select(Numbers).ToArray(),Numbers(shape.GetProperty("direction")),shape.GetProperty("depthM").GetDouble());bodies.Add(body);body.Name=name;
                var pairs=tubes.Select(t=>Intersection(t,body,t.Name,name)).ToArray();
                reports.Add(new {name,continuousPath=true,method="Native Boolean intersection with full declared prism (conservative envelopes identified in layout recipe)",volumeM3=Volume(body),boundsM=body.GetBodyBox(),accepted=pairs.All(x=>x.resolved&&x.overlapM3<=1e-12),pairs});
            }
            foreach(var circle in layout.GetProperty("driverCircles").EnumerateArray()) {
                string name=circle.GetProperty("name").GetString()!;var center=Numbers(circle.GetProperty("centerM"));double radius=circle.GetProperty("radiusM").GetDouble();
                // A circumscribed 128-gon contains the entire exact rule circle.
                // Passing it is a conservative clearance proof, never an undersized template.
                var pts=Enumerable.Range(0,128).Select(i=>new[]{center[0]+radius/Math.Cos(Math.PI/128)*Math.Cos(2*Math.PI*i/128),-.0035,center[2]+radius/Math.Cos(Math.PI/128)*Math.Sin(2*Math.PI*i/128)}).ToArray();
                var body=PolygonPrism(pts,new double[]{0,1,0},.007);bodies.Add(body);body.Name=name;
                var pairs=tubes.Select(t=>Intersection(t,body,t.Name,name)).ToArray();
                reports.Add(new {name,method="Native conservative circumscribed 128-gon, 7 mm thick; exact nominal dimensions checked separately",radiusM=radius,maximumRadialExcessM=radius*(1/Math.Cos(Math.PI/128)-1),accepted=pairs.All(x=>x.resolved&&x.overlapM3<=1e-12),pairs});
            }
            string? previewModel=null;
            if(job.TryGetProperty("output",out var outPath)) {
                previewModel=Safe(outPath.GetString()!);var proof=New(false);
                try {for(int i=0;i<bodies.Count;i++){var f=((PartDoc)proof).CreateFeatureFromBody3(bodies[i],false,0) as Feature??throw new Exception("Proof feature");f.Name="CheckEnvelope_"+i;}SaveNew(proof,previewModel);}finally{App.CloseDoc(proof.GetTitle());}
            }
            return new {path=d.GetPathName(),cadSha256=FileSha(d.GetPathName()),layoutRecipeSha256=FileSha(job.GetProperty("layoutRecipe").GetString()!),reports,previewModel,
                scope="Classroom frame and explicitly proposed layout datums; no claim about unmodeled team hardware or actual-driver certification"};
        } finally {foreach(var b in bodies)Marshal.ReleaseComObject(b);if(job.TryGetProperty("restore",out var restore))Open(restore.GetString()!);}
    }
}
