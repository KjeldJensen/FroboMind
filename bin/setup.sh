#!/usr/bin/env sh
#*****************************************************************************
# FroboMind setup script
# Copyright (c) 2014-2016, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
# This bash setup script adds a path to the frobomind_make script and enables
# auto completion.
#
# 2016-10-31 KJ First version
#
#*****************************************************************************

# find out where the FroboMind bin dir is
_ROOT_FM_BIN=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

# add to the path
export PATH="$PATH:$_ROOT_FM_BIN"

# enable tab auto completion for the fm_make script
_script()
{
  _ROOT_FM_BIN=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
  cd $_ROOT_FM_BIN/../..
  PACKAGES=`find $ROOT_SRC -name "fmMake.txt"` # find all directories with fmMake.txt
  _script_commands=''
  for LINE in $PACKAGES; do # for each directory
    fmmake_dir=$(dirname $LINE)
    fmmake_dir_stripped=${fmmake_dir:1}
    _script_commands=$_script_commands$IFS$fmmake_dir_stripped
  done 

  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "${_script_commands}" -- ${cur}) )

  return 0
}
complete -o nospace -F _script fm_make

