# Script obtained from the https://github.com/Homebrew/homebrew-science repository

'''
Copyright 2009-2016 Homebrew contributors.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

class Scotch < Formula
    desc "Graph and mesh partitioning, clustering, and sparse matrix ordering"
    homepage "https://gforge.inria.fr/projects/scotch"
    url "https://gforge.inria.fr/frs/download.php/file/34618/scotch_6.0.4.tar.gz"
    sha256 "f53f4d71a8345ba15e2dd4e102a35fd83915abf50ea73e1bf6efe1bc2b4220c7"
    
    option "without-test", "skip build-time tests (not recommended)"
    
    depends_on "bzip2" unless OS.mac?
    depends_on "open-mpi"
    depends_on "xz" => :optional # Provides lzma compression.
    
    def install
        ENV.deparallelize if MacOS.version >= :sierra
        cd "src" do
            ln_s "Make.inc/Makefile.inc.i686_mac_darwin10", "Makefile.inc"
            # default CFLAGS:
            # -O3 -Drestrict=__restrict -DCOMMON_FILE_COMPRESS_GZ -DCOMMON_PTHREAD
            # -DCOMMON_PTHREAD_BARRIER -DCOMMON_RANDOM_FIXED_SEED -DCOMMON_TIMING_OLD
            # -DSCOTCH_PTHREAD -DSCOTCH_RENAME
            # MPI implementation is not threadsafe, do not use DSCOTCH_PTHREAD
            
            cflags   = %w[-O3 -fPIC -Drestrict=__restrict -DCOMMON_PTHREAD_BARRIER
            -DCOMMON_PTHREAD
            -DSCOTCH_CHECK_AUTO -DCOMMON_RANDOM_FIXED_SEED
            -DCOMMON_TIMING_OLD -DSCOTCH_RENAME
            -DCOMMON_FILE_COMPRESS_BZ2 -DCOMMON_FILE_COMPRESS_GZ]
            ldflags  = %w[-lm -lz -lpthread -lbz2]
            
            cflags  += %w[-DCOMMON_FILE_COMPRESS_LZMA]   if build.with? "xz"
            ldflags += %W[-L#{Formula["xz"].lib} -llzma] if build.with? "xz"
            
            make_args = ["CCS=#{ENV["CC"]}",
            "CCP=mpicc",
            "CCD=mpicc",
            "RANLIB=echo",
            "CFLAGS=#{cflags.join(" ")}",
            "LDFLAGS=#{ldflags.join(" ")}"]
            
            if OS.mac?
                make_args << "LIB=.dylib"
                make_args << "AR=libtool"
                arflags = ldflags.join(" ") + " -dynamic -install_name #{lib}/$(notdir $@) -undefined dynamic_lookup -o"
                make_args << "ARFLAGS=#{arflags}"
                else
                make_args << "LIB=.so"
                make_args << "ARCH=ar"
                make_args << "ARCHFLAGS=-ruv"
            end
            
            system "make", "scotch", "VERBOSE=ON", *make_args
            system "make", "ptscotch", "VERBOSE=ON", *make_args
            system "make", "install", "prefix=#{prefix}", *make_args
            system "make", "check", "ptcheck", "EXECP=mpirun -np 2", *make_args if build.with? "test"
        end
        
        # Install documentation + sample graphs and grids.
        doc.install Dir["doc/*.pdf"]
        pkgshare.install Dir["doc/*.f"], Dir["doc/*.txt"]
        pkgshare.install "grf", "tgt"
    end
    
    test do
    mktemp do
    system "echo cmplt 7 | #{bin}/gmap #{pkgshare}/grf/bump.grf.gz - bump.map"
    system "#{bin}/gmk_m2 32 32 | #{bin}/gmap - #{pkgshare}/tgt/h8.tgt brol.map"
    system "#{bin}/gout", "-Mn", "-Oi", "#{pkgshare}/grf/4elt.grf.gz", "#{pkgshare}/grf/4elt.xyz.gz", "-", "graph.iv"
end
end
end
