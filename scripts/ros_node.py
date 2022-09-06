#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

if __name__ == '__main__':
    #example_function()
    plugin = 'rqt_mypkg'
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))
