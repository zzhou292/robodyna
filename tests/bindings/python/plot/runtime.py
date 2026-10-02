"""Actual packaged NumPy/Matplotlib extension imports and CPU Agg image output."""
import argparse,importlib,json,os,sys,tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from origin import require_declared_origin

parser=argparse.ArgumentParser();parser.add_argument('--package-manifest',type=Path,required=True);args=parser.parse_args()
manifest=args.package_manifest.absolute();document=json.loads(manifest.read_text());sys.path.insert(0,str(manifest.parent));product=importlib.import_module('robodyna')
with tempfile.TemporaryDirectory(dir=os.environ.get('TEST_TMPDIR')) as temporary:
    root=Path(temporary);os.environ['MPLCONFIGDIR']=str(root/'config');os.environ['MPLBACKEND']='Agg'
    import numpy as np
    import matplotlib
    require_declared_origin(np, manifest, document, 'numpy/__init__.py')
    require_declared_origin(matplotlib, manifest, document, 'matplotlib/__init__.py')
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    if np.__version__!='1.26.4' or matplotlib.__version__!='3.8.4':raise RuntimeError('Unqualified plotting/runtime version')
    x=np.linspace(0,1,101);y=x*x
    fig,ax=plt.subplots(figsize=(4,3),dpi=100);line,=ax.plot(x,y)
    if not np.array_equal(line.get_ydata(),y):raise RuntimeError('Plot array conversion differs')
    fig.savefig(root/'plot.png');fig.savefig(root/'plot.pdf');plt.close(fig)
    with Image.open(root/'plot.png') as image:
        image.load()
        if image.size!=(400,300) or not any(high>low for low,high in image.convert('RGB').getextrema()):raise RuntimeError('Actual Agg PNG is invalid')
    if not (root/'plot.pdf').read_bytes().startswith(b'%PDF-'):raise RuntimeError('Actual PDF export failed')
    native=product.ChVector3d(1,2,3).to_numpy()
    if not np.array_equal(native,[1,2,3]):raise RuntimeError('Native core/NumPy bridge differs')
print(json.dumps({'passed':True,'scope':'Actual packaged native array and Agg PNG/PDF output; no interactive GUI or physics demo qualification','matplotlib':'3.8.4','numpy':'1.26.4','gpu_calls':False}))
