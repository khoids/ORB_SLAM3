python
import sys
sys.path.insert(0, '/home/koi/eigen-3.4.0/debug/gdb')  #In my case it was /home/pc/eigen/debug/gdb
from printers import register_eigen_printers
register_eigen_printers (None)
end