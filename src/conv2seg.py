#! /usr/bin/python3
##
#
import argparse
import sys
import re

verbose=False

seg16= [ '_DP','_U ','_T ','_S ','_R ','_P ','_N ','_M ','_K ','_H ',
         '_G ','_F ','_E ','_D ','_C ','_B ','_A ',]
seg14= [ '_DP', '_L ','_M ','_N ','_K ','_J ','_H ','_G2',
         '_G1','_F ','_E ','_D ','_C ','_B ','_A ',]
seg7 = [ '_DP', '_G ','_F ','_E ','_D ','_C ','_B ','_A ']
filler='   '

nsegs=14
segtable=seg14


def printsymbols():
    print("#include <stdint.h>")
    for i in range(0,nsegs+1):
        print('#define {0} 0x{1:x}'.format(segtable[i],2**(nsegs-i)))

def processfile(filename):

    if verbose:
        print(filename)
    firstspace=True
    with open(filename,'r') as f:
        line=f.readline()
        while line:
            if verbose:
                print(line)
            m = re.search('.*0b([01]+)',line)
            if m:
                segs = m.group(1)
                if verbose:
                    print("segs={}    size={}".format(segs,len(segs)))
                sep=" "
                newsegs=""
                for i in range(0,len(segs)):
                    if segs[i]=='1':
                        newsegs+=sep+segtable[i]
                        sep='|';
                    else:
                        newsegs+=' '+filler
                if newsegs.isspace():
                    newsegs = " 0"+filler*(nsegs+1)+' '*(nsegs-1);

                if verbose:
                    print(newsegs)
                nl=line.replace('0b'+segs,newsegs)
                print(nl,end='')
            else:
                print(line,end='')
            line=f.readline()
            if firstspace and line.isspace():
                printsymbols()
                firstspace=False

def main():
    global nsegs
    global segtable

    parser = argparse.ArgumentParser(description='Process files .')

    parser.add_argument('--mode', type=int, choices=[7,14,16],help='mode',default=14)
    parser.add_argument('--verbose','-v',dest='verbose', action='store_true',
                   help='verbose')
    parser.add_argument('files',nargs='+')


    args = parser.parse_args()
    verbose=args.verbose

    if verbose:
        print("mode={}".format(args.mode))

    if args.mode == 16:
        segtable=seg16
        nsegs = 16
    elif args.mode == 7:
        segtable = seg7
        nsegs = 7

    for f in args.files:
        processfile(f)

if __name__ == '__main__':
    sys.exit(main())

