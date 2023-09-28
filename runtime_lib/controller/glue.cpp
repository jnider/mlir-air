//
// Copyright (C) 2023, Advanced Micro Devices, Inc.
// SPDX-License-Identifier: MIT
//

/*
  These stubs are needed to compile libc
  They are not intended to ever be called
*/

extern "C" void _exit(int)
{
  while(1);
}

extern "C" void _sbrk(void)
{
}

extern "C" int _write(int fd, char* buf, int nbytes)
{
  return 0;
}

extern "C" void _close(void)
{
}

extern "C" void _fstat(void)
{
}

extern "C" void _isatty(void)
{
}

extern "C" void _lseek(void)
{
}

extern "C" void _read(void)
{
}
