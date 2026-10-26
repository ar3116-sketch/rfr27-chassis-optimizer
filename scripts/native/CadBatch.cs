using SolidWorks.Interop.sldworks;

internal static partial class Program
{
    // Transient API batching only. Restore every setting even on a rejected cut.
    sealed class CadBatch : IDisposable {
        readonly ModelDoc2 doc;readonly ModelView? view;readonly bool graphics,tree,command;bool disposed;
        public CadBatch(ModelDoc2 d) {
            doc=d;view=d.ActiveView as ModelView;graphics=view?.EnableGraphicsUpdate??true;tree=d.FeatureManager.EnableFeatureTree;command=App.CommandInProgress;
            App.CommandInProgress=true;d.FeatureManager.EnableFeatureTree=false;if(view!=null)view.EnableGraphicsUpdate=false;
        }
        public void Dispose() {
            if(disposed)return;disposed=true;
            try{doc.FeatureManager.EnableFeatureTree=tree;if(view!=null)view.EnableGraphicsUpdate=graphics;}finally{App.CommandInProgress=command;}
        }
    }
}
