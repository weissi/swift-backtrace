#ifdef __linux__
/* mmapio.c -- File views using mmap.
   Copyright (C) 2012-2018 Free Software Foundation, Inc.
   Written by Ian Lance Taylor, Google.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    (1) Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    (2) Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.

    (3) The name of the author may not be used to
    endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.  */

#include "config.h"

#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>

#include "include/backtrace.h"
#include "internal.h"

#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif

/* This file implements file views and memory allocation when mmap is
   available.  */

/* Create a view of SIZE bytes from DESCRIPTOR at OFFSET.  */

int
backtrace_get_view (struct backtrace_state *state ATTRIBUTE_UNUSED,
		    int descriptor, off_t offset, size_t size,
		    backtrace_error_callback error_callback,
		    void *data, struct backtrace_view *view)
{
  size_t pagesize;
  unsigned int inpage;
  off_t pageoff;
  void *map;

  pagesize = getpagesize ();
  inpage = offset % pagesize;
  pageoff = offset - inpage;

  size += inpage;
  size = (size + (pagesize - 1)) & ~ (pagesize - 1);

  map = mmap (NULL, size, PROT_READ, MAP_PRIVATE, descriptor, pageoff);
  if (map == MAP_FAILED)
    {
        int mmap_errno = errno;
        struct stat stat = {0};
        char *msg = NULL;
        char filename[1024] = {0};
	char binary[1024] = {0};

        if (fstat(descriptor, &stat) == -1) {
                error_callback (data, "fstat", errno);
        }

        asprintf(&msg, "/proc/self/fd/%d", descriptor);
        if (readlink(msg, filename, 1024) == -1) {
                if (errno == ENOENT) {
                        memcpy(filename, "not found", 10);
                } else {
                        error_callback (data, "readlink", errno);
                }
        }
        free(msg);
        msg = NULL;

	readlink("/proc/self/exe", binary, 1024);

        asprintf(&msg, "filename=%s, binary=%s, fd=%d, size=%ld, mode=%d, rdev=%lu, dev=%lu, ino=%lu; mmap:size=%ld, mmap:offset=%ld",
                 filename, binary, descriptor, stat.st_size, stat.st_mode, stat.st_rdev, stat.st_dev, stat.st_ino, size, pageoff);
        error_callback (data, msg, mmap_errno);
        free(msg);
        msg = NULL;

        error_callback (data, "mmap file i/o", mmap_errno);

        return 0;
    }

  view->data = (char *) map + inpage;
  view->base = map;
  view->len = size;

  return 1;
}

/* Release a view read by backtrace_get_view.  */

void
backtrace_release_view (struct backtrace_state *state ATTRIBUTE_UNUSED,
			struct backtrace_view *view,
			backtrace_error_callback error_callback,
			void *data)
{
  union {
    const void *cv;
    void *v;
  } const_cast;

  const_cast.cv = view->base;
  if (munmap (const_cast.v, view->len) < 0)
    error_callback (data, "munmap", errno);
}
#endif
