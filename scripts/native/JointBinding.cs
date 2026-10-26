using System.Diagnostics;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

internal static partial class Program
{
    sealed record BoundJoint(object entity,string nativeName,double[] requestedPointM,double selectionSeconds,double[]? selectedPointM=null,double offsetM=0);
    static BoundJoint[] BindBeamJoints(ModelDoc2 d,CWStudy study,double[][] points,double[][][]? searchDirs=null,ISet<int>? preferSide=null,double? locateReach=null) {
        var cw=RequireSimulation();var manager=cw.ActiveDoc.StudyManager;
        if(manager.GetStudy(manager.ActiveStudy).Name!=study.Name)throw new Exception("Wrong active study for joint binding");
        var callback=(ICwAddincallback)App.GetAddInObject("SldWorks.Simulation."+(int.Parse(App.RevisionNumber().Split('.')[0])-15));
        var selected=new List<BoundJoint>();var sm=(SelectionMgr)d.SelectionManager;var missing=new List<int>();
        Stage("Entering native fixture-selection context");callback.DefineRestraints();
        try {
            // Pass 1, as the procedure's manual pick: select at each point in the study's own view,
            // walking out along incident members for trimmed joints. Pass 2 re-picks each point in
            // the *Top (left points) or *Bottom (right) view zoomed around it. (In the team part
            // these look along the car's vertical axis, not from its side: their orientation is
            // logged by joint-pick-test.) A pick ignores depth (see PickJoint), so either pass can
            // return a joint that only projects near the point: an X-brace crossing at the
            // centreline for a rear pickup, or the front-hoop-base joint 28.7 mm below the front
            // lower-aft pickup (FA0), whose own joint is 4.7 mm outboard on the hoop axis.
            // The first view is kept unless it found nothing or validation sent the point back
            // (preferSide). Pass 3 (locateReach) then locates each pick in 3D and replaces only a
            // pick it cannot place within reach, by the nearest joint it can.
            object? lastEntity=null;
            string? Pick(double[] q){d.ClearSelection2(true);lastEntity=null;
                if(!d.Extension.SelectByID2("","SIMULATION",q[0],q[1],q[2],false,0,null,0))return null;
                var o=sm.GetSelectedObject6(1,-1);
                if(o is IManipulator mm&&mm.Name.StartsWith("Joint<")){lastEntity=o;return mm.Name;}
                return null;}
            var first=new (string? name,object? entity,double[] at,double offset,double seconds)[points.Length];
            for(int pi=0;pi<points.Length;pi++) {
                var p=points[pi];var timer=Stopwatch.StartNew();
                string? n=Pick(p);object? e=lastEntity;double[] at=p;double offset=0;
                if(n==null&&searchDirs!=null&&searchDirs[pi].Length>0)
                    for(double t=.0005;t<=.04&&n==null;t+=.0005)
                        foreach(var u in searchDirs[pi]) {
                            var q=new[]{p[0]+t*u[0],p[1]+t*u[1],p[2]+t*u[2]};
                            if((n=Pick(q))!=null){e=lastEntity;at=q;offset=t;break;}
                        }
                first[pi]=(n,e,at,offset,timer.Elapsed.TotalSeconds);
            }
            // Per point: joint name, entity, the point where it was picked (pass 3: the located joint
            // centre), that point's distance from the requested one, and how it was chosen.
            var chosen=new (string? name,object? entity,double[] at,double offset,string how)[points.Length];
            for(int pi=0;pi<points.Length;pi++) {
                var p=points[pi];var f=first[pi];
                d.ShowNamedView2(p[1]>=0?"*Top":"*Bottom",-1);
                const double h=.08;d.ViewZoomTo2(p[0]-h,p[1]-h,p[2]-h,p[0]+h,p[1]+h,p[2]+h);
                string? side=Pick(p);object? sideEntity=lastEntity;
                // Default: the first view (the manual pick). The side view is used where the first
                // view found nothing, or for points the result validation sent back (preferSide).
                bool useSide=side!=null&&(f.name==null||(preferSide?.Contains(pi)??false));
                string? name=useSide?side:f.name;object? entity=useSide?sideEntity:f.entity;
                if(side!=null&&f.name!=null&&side!=f.name)Stage($"Joint pick disagreement at point {pi}: first view {f.name}, side view {side} ({(useSide?"side":"first")} view used)");
                bool sameAsFirst=!useSide||side==f.name;
                chosen[pi]=(name,entity,sameAsFirst?f.at:p,sameAsFirst?f.offset:0,side==f.name?"views agree":useSide?"side view":"first view");
            }
            if(locateReach is double reach) {
                var views=AxisViews(d);
                for(int pi=0;pi<points.Length;pi++) {
                    var c=chosen[pi];var timer=Stopwatch.StartNew();
                    var (found,confirmed)=LocalizeJoints(d,views,points[pi],reach,c.name);
                    if(confirmed!=null){Stage($"Located {c.name} {confirmed.distanceM*1000:0.0} mm from point {pi} ({timer.Elapsed.TotalSeconds:0.0} s)");continue;}
                    if(found.Count==0){Stage($"Point {pi}: no joint located within {reach*1000:0} mm; pick {c.name??"(none)"} kept for validation");continue;}
                    var best=found[0];
                    Stage($"Point {pi}: pick {c.name??"(none)"} not located within {reach*1000:0} mm; nearest located joint {best.name} at {best.distanceM*1000:0.0} mm used ({timer.Elapsed.TotalSeconds:0.0} s)");
                    chosen[pi]=(best.name,best.entity,points[pi].Zip(best.offsetM,(x,y)=>x+y).ToArray(),best.distanceM,"located");
                }
            }
            for(int pi=0;pi<points.Length;pi++) {
                var p=points[pi];var c=chosen[pi];
                if(c.name==null){missing.Add(pi);Stage($"MISS point {pi}: {string.Join(",",p.Select(x=>x.ToString("0.0000")))}");continue;}
                if(c.entity is not IManipulator m||!m.Name.StartsWith("Joint<"))throw new Exception("Selection is not a beam joint");
                if(selected.Any(x=>x.nativeName==m.Name))throw new Exception($"Requested distinct points mapped to the same joint {m.Name} (point {pi})");
                selected.Add(new(c.entity,m.Name,p,first[pi].seconds,c.at,c.offset));
                Stage($"Bound {m.Name} offset {c.offset*1000:0.0} mm ({c.how})");
            }
        } finally {
            d.ShowNamedView2("*Isometric",-1);
            // API cancellation closes the selection context, never a keyboard/mouse input.
            if(!App.RunCommand(3043,""))throw new Exception("Could not close native fixture-selection context");
            d.ClearSelection2(true);
        }
        if(missing.Count>0)throw new Exception($"No native joint found for points {string.Join(",",missing)} of {points.Length}");
        if(manager.GetStudy(manager.ActiveStudy).Name!=study.Name)throw new Exception("Study changed during joint binding");
        return selected.ToArray();
    }
    // SelectByID2 on a beam joint ignores depth: it returns a joint whose screen projection lies
    // within about 1.3 mm (at an 80 mm zoom half-size) of the point's projection, wherever that
    // joint lies along the line of sight; where several overlap it returns one of them (the
    // lowest-numbered in the mirror-overlap cases seen). Evidence: engineering-2026-10-12T16-37-00-873Z
    // and T16-40-07-150Z (joint-pick-test on the 009c study).
    static string? PickJoint(ModelDoc2 d,double[] q,out object? entity) {
        entity=null;d.ClearSelection2(true);
        if(!d.Extension.SelectByID2("","SIMULATION",q[0],q[1],q[2],false,0,null,0))return null;
        var o=((SelectionMgr)d.SelectionManager).GetSelectedObject6(1,-1);
        if(o is IManipulator m&&m.Name.StartsWith("Joint<")){entity=o;return m.Name;}
        return null;
    }
    // Screen axes and line of sight of the active view in model coordinates (Orientation3 columns).
    static (double[] u,double[] v,double[] n) ViewAxes(ModelDoc2 d) {
        var a=(double[])((ModelView)d.ActiveView).Orientation3.ArrayData;
        return (Unit(new[]{a[0],a[3],a[6]}),Unit(new[]{a[1],a[4],a[7]}),Unit(new[]{a[2],a[5],a[8]}));
    }
    // The standard view whose line of sight is closest to each model axis, chosen by orientation,
    // not name: in the team part *Top looks down the car's vertical (z) axis.
    static string[] AxisViews(ModelDoc2 d) {
        var best=new string[3];var score=new double[3];
        foreach(var name in new[]{"*Front","*Back","*Left","*Right","*Top","*Bottom"}) {
            d.ShowNamedView2(name,-1);var n=ViewAxes(d).n;
            for(int k=0;k<3;k++)if(Math.Abs(n[k])>score[k]){score[k]=Math.Abs(n[k]);best[k]=name;}
        }
        if(score.Any(x=>x<.99))throw new Exception("Standard views are not axis-aligned: "+string.Join(",",score));
        return best;
    }
    sealed record JointCandidate(string name,object entity,double[] offsetM,double distanceM,int planHits,int depthHits);
    // Locate the beam joints within reach of p. A pick cannot see depth, so: (1) in the plan view
    // (line of sight = car z) a screen grid around p, walked outward by radius, finds every joint
    // that projects within reach; (2) in the longitudinal view (line of sight = car x, where left
    // and right joints do not overlap) a sweep along the plan view's line of sight through each
    // joint's plan position finds its depth. A joint is located once the grid has passed its whole
    // plan footprint. Unless exhaustive, the walk stops when `confirm` is located within reach, or
    // when it is past the nearest located joint (an unseen joint is at least its plan radius away).
    static (List<JointCandidate> found,JointCandidate? confirmed) LocalizeJoints(ModelDoc2 d,string[] axisViews,double[] p,double reach,string? confirm=null,bool exhaustive=false) {
        const double h=.08,step=.001,footprint=.003;double r=reach+.001;int m=(int)Math.Round(r/step);
        double[] At(double[] u,double a,double[] v,double b,double[] n,double w)=>new[]{p[0]+a*u[0]+b*v[0]+w*n[0],p[1]+a*u[1]+b*v[1]+w*n[1],p[2]+a*u[2]+b*v[2]+w*n[2]};
        void Show(int axis){d.ShowNamedView2(axisViews[axis],-1);d.ViewZoomTo2(p[0]-h,p[1]-h,p[2]-h,p[0]+h,p[1]+h,p[2]+h);}
        Show(2);var (u,v,n)=ViewAxes(d);
        var grid=(from i in Enumerable.Range(-m,2*m+1) from j in Enumerable.Range(-m,2*m+1)
                  let a=i*step let b=j*step let rho=Math.Sqrt(a*a+b*b) where rho<=r orderby rho select (a,b,rho)).ToArray();
        var hits=new Dictionary<string,(object entity,List<double[]> at,double first)>();var done=new HashSet<string>();var found=new List<JointCandidate>();
        JointCandidate? Locate(string name) {
            var hit=hits[name];done.Add(name);
            double ca=hit.at.Average(x=>x[0]),cb=hit.at.Average(x=>x[1]);var depths=new List<double>();
            Show(0);for(int k=-m;k<=m;k++)if(PickJoint(d,At(u,ca,v,cb,n,k*step),out _)==name)depths.Add(k*step);Show(2);
            if(depths.Count==0)return null;
            var off=At(u,ca,v,cb,n,depths.Average()).Zip(p,(x,y)=>x-y).ToArray();
            if(Norm(off)>reach)return null;
            var c=new JointCandidate(name,hit.entity,off,Norm(off),hit.at.Count,depths.Count);found.Add(c);return c;
        }
        foreach(var (a,b,rho) in grid) {
            if(!exhaustive&&found.Count>0&&rho>found.Min(c=>c.distanceM)+step&&hits.Keys.All(done.Contains))break;
            var name=PickJoint(d,At(u,a,v,b,n,0),out var e);
            if(name!=null){if(!hits.ContainsKey(name))hits[name]=(e!,new List<double[]>(),rho);hits[name].at.Add(new[]{a,b});}
            foreach(var k in hits.Keys.Where(k=>!done.Contains(k)&&rho>hits[k].first+footprint).ToList())
                if(Locate(k) is JointCandidate c&&!exhaustive&&k==confirm)return (found,c);
        }
        foreach(var k in hits.Keys.Where(k=>!done.Contains(k)).ToList())Locate(k);
        return (found.OrderBy(c=>c.distanceM).ToList(),found.FirstOrDefault(c=>c.name==confirm));
    }
    static object[] ReadBoundJointNodes(CWMesh mesh,BoundJoint[] joints) => joints.Select(j=>(object)new {
        j.nativeName,j.requestedPointM,j.selectedPointM,j.offsetM,j.selectionSeconds,
        verification="Verify native deck GRID/SPC/FORCE geometry and solved result coordinates; joint GetNodeDataFromEntity is unsupported"
    }).ToArray();
    static object DirectionPlane(ModelDoc2 d,string name,out object transform) {
        var f=Find(d,name)??throw new Exception("Direction plane absent: "+name);transform=((RefPlane)f.GetSpecificFeature2()).Transform.ArrayData;
        d.ClearSelection2(true);if(!f.Select2(false,0))throw new Exception("Direction plane selection failed");
        var result=((SelectionMgr)d.SelectionManager).GetSelectedObject6(1,-1);d.ClearSelection2(true);return result;
    }
}
