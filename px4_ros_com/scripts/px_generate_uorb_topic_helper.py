#!/usr/bin/env python3
#############################################################################
#
#   Copyright (C) 2013-2021 PX4 Pro Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

'''
Helper methods & common code for the uorb message templates msg.{cpp,h}.em

Another positive effect of having the code here, is that this file will get
precompiled and thus message generation will be much faster
'''

import os
import errno

import genmsg.msgs


type_map = {
    'int8': 'int8_t',
    'int16': 'int16_t',
    'int32': 'int32_t',
    'int64': 'int64_t',
    'uint8': 'uint8_t',
    'uint16': 'uint16_t',
    'uint32': 'uint32_t',
    'uint64': 'uint64_t',
    'float32': 'float',
    'float64': 'double',
    'bool': 'bool',
    'char': 'char',
}

type_map_short = {
    # We use some range outside of alpha-numeric and special characters.
    # This needs to match with orb_get_c_type()
    'int8': '\\x82',
    'int16': '\\x83',
    'int32': '\\x84',
    'int64': '\\x85',
    'uint8': '\\x86',
    'uint16': '\\x87',
    'uint32': '\\x88',
    'uint64': '\\x89',
    'float32': '\\x8a',
    'float64': '\\x8b',
    'bool': '\\x8c',
    'char': '\\x8d',
}

type_serialize_map = {
    'int8': 'int8_t',
    'int16': 'int16_t',
    'int32': 'int32_t',
    'int64': 'int64_t',
    'uint8': 'uint8_t',
    'uint16': 'uint16_t',
    'uint32': 'uint32_t',
    'uint64': 'uint64_t',
    'float32': 'float',
    'float64': 'double',
    'bool': 'bool',
    'char': 'char',
}

type_idl_map = {
    'bool': 'boolean',
    'byte': 'octet',
    'char': 'char',
    'int8': 'octet',
    'uint8': 'octet',
    'int16': 'short',
    'uint16': 'unsigned short',
    'int32': 'long',
    'uint32': 'unsigned long',
    'int64': 'long long',
    'uint64': 'unsigned long long',
    'float32': 'float',
    'float64': 'double',
    'string': 'string',
}

msgtype_size_map = {
    'int8': 1,
    'int16': 2,
    'int32': 4,
    'int64': 8,
    'uint8': 1,
    'uint16': 2,
    'uint32': 4,
    'uint64': 8,
    'float32': 4,
    'float64': 8,
    'bool': 1,
    'char': 1,
}

type_printf_map = {
    'int8': '%d',
    'int16': '%d',
    'int32': '%" PRId32 "',
    'int64': '%" PRId64 "',
    'uint8': '%u',
    'uint16': '%u',
    'uint32': '%" PRIu32 "',
    'uint64': '%" PRIu64 "',
    'float32': '%.4f',
    'float64': '%.6f',
    'bool': '%u',
    'char': '%c',
}


def bare_name(msg_type):
    """
    Get bare_name from <dir>/<bare_name>[x] format
    """
    bare = msg_type
    if '/' in msg_type:
        # removing prefix
        bare = (msg_type.split('/'))[1]
    # removing suffix
    return bare.split('[')[0]


def sizeof_field_type(field):
    """
    Get size of a field, used for sorting
    """
    bare_name_str = bare_name(field.type)
    if bare_name_str in msgtype_size_map:
        return msgtype_size_map[bare_name_str]
    return 0  # this is for non-builtin types: sort them at the end


def get_children_fields(base_type, search_path):
    (package, name) = genmsg.names.package_resource_name(base_type)
    tmp_msg_context = genmsg.msg_loader.MsgContext.create_default()
    spec_temp = genmsg.msg_loader.load_msg_by_type(
        tmp_msg_context, '%s/%s' % (package, name), search_path)
    return spec_temp.parsed_fields()


def add_padding_bytes(fields, search_path):
    """
    Add padding fields before the embedded types, at the end and calculate the
    struct size
    returns a tuple with the struct size and padding at the end
    """
    struct_size = 0
    align_to = 8  # this is always 8, because of the 64bit timestamp
    i = 0
    padding_idx = 0
    while i < len(fields):
        field = fields[i]
        if not field.is_header:
            a_pos = field.type.find('[')
            array_size = 1
            if field.is_array:
                array_size = field.array_len
            if field.is_builtin:
                field.sizeof_field_type = sizeof_field_type(field)
            else:
                # embedded type: may need to add padding
                num_padding_bytes = align_to - (struct_size % align_to)
                if num_padding_bytes != align_to:
                    padding_field = genmsg.Field('_padding' + str(padding_idx),
                                                 'uint8[' + str(num_padding_bytes) + ']')
                    padding_idx += 1
                    padding_field.sizeof_field_type = 1
                    struct_size += num_padding_bytes
                    fields.insert(i, padding_field)
                    i += 1
                children_fields = get_children_fields(
                    field.base_type, search_path)
                field.sizeof_field_type, unused = add_padding_bytes(children_fields,
                                                                    search_path)
            struct_size += field.sizeof_field_type * array_size
        i += 1

    # add padding at the end (necessary for embedded types)
    num_padding_bytes = align_to - (struct_size % align_to)
    if num_padding_bytes == align_to:
        num_padding_bytes = 0
    else:
        padding_field = genmsg.Field('_padding' + str(padding_idx),
                                     'uint8[' + str(num_padding_bytes) + ']')
        padding_idx += 1
        padding_field.sizeof_field_type = 1
        struct_size += num_padding_bytes
        fields.append(padding_field)
    return (struct_size, num_padding_bytes)


def convert_type(spec_type, use_short_type=False):
    """
    Convert from msg type to C type
    """
    bare_type = spec_type
    if '/' in spec_type:
        # removing prefix
        bare_type = (spec_type.split('/'))[1]

    msg_type, is_array, array_length = genmsg.msgs.parse_type(bare_type)
    c_type = msg_type
    if use_short_type and msg_type in type_map_short:
        c_type = type_map_short[msg_type]
    elif msg_type in type_map:
        c_type = type_map[msg_type]
    if is_array:
        return c_type + "[" + str(array_length) + "]"
    return c_type


def print_field_def(field):
    """
    Print the C type from a field
    """

    # check if there are any upper case letters in the field name
    assert not any(a.isupper()
                   for a in field.name), "%r field contains uppercase letters" % field.name

    type_name = field.type
    # detect embedded types
    sl_pos = type_name.find('/')
    type_appendix = ''
    type_prefix = ''
    if (sl_pos >= 0):
        type_name = type_name[sl_pos + 1:]
        type_prefix = 'struct '
        type_appendix = '_s'

    # detect arrays
    a_pos = type_name.find('[')
    array_size = ''
    if (a_pos >= 0):
        # field is array
        array_size = type_name[a_pos:]
        type_name = type_name[:a_pos]

    if type_name in type_map:
        # need to add _t: int8 --> int8_t
        type_px4 = type_map[type_name]
    else:
        type_px4 = type_name

    comment = ''
    if field.name.startswith('_padding'):
        comment = ' // required for logger'

    print(('\t%s%s%s %s%s;%s' % (type_prefix, type_px4, type_appendix, field.name,
                                 array_size, comment)))
