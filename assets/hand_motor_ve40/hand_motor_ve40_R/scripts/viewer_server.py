from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys,json,threading,argparse
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from solve_closed_chain import ClosureSolver
solver=ClosureSolver(ROOT/'hand_tree_only.urdf',ROOT/'config/loop_closures.yaml')
lock=threading.Lock();namefile=ROOT/'config/custom_motor_names.json'
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*a,**kw):super().__init__(*a,directory=str(ROOT),**kw)
    def sendjson(self,data,status=200):
        b=json.dumps(data,ensure_ascii=False).encode('utf8');self.send_response(status)
        self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    def do_GET(self):
        if self.path=='/api/state':
            with lock:self.sendjson(dict(independent=solver.independent,positions=solver.state,gap_m=solver.max_gap(solver.state),tolerance_m=solver.tolerance))
        elif self.path=='/api/names':self.sendjson({'names':json.loads(namefile.read_text(encoding='utf8')) if namefile.exists() else {}})
        else:super().do_GET()
    def do_POST(self):
        if self.path not in ['/api/pose','/api/reset','/api/names']:return self.send_error(404)
        try:
            size=int(self.headers.get('Content-Length','0'))
            if size>16384:raise ValueError('Request too large')
            data=json.loads(self.rfile.read(size) or b'{}')
            with lock:
                if self.path=='/api/names':
                    if data.get('joint') not in solver.independent:raise ValueError('Unknown motor')
                    value=str(data.get('name','')).strip()
                    if len(value)>60:raise ValueError('Name too long')
                    names=json.loads(namefile.read_text(encoding='utf8')) if namefile.exists() else {}
                    if value:names[data['joint']]=value
                    else:names.pop(data['joint'],None)
                    namefile.write_text(json.dumps(names,ensure_ascii=False,indent=2),encoding='utf8');return self.sendjson({'names':names})
                try:
                    if self.path=='/api/reset':solver.reset()
                    else:solver.solve(data)
                    self.sendjson(dict(positions=solver.state,gap_m=solver.max_gap(solver.state)))
                except ValueError as e:self.sendjson(dict(error=str(e),positions=solver.state,gap_m=solver.max_gap(solver.state)),422)
        except Exception as e:self.sendjson({'error':str(e)},400)
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8098);args=parser.parse_args()
    print(f'Preview: http://localhost:{args.port}/viewer/index.html',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
