import xml.etree.ElementTree as ET
from time import localtime, strftime

from numpy import full

vars = ET.parse('datavars.xml')

var_config = vars.getroot()

def decode_debugoutputs(xmlitem):
    fulldebugarray = []
    debugnum = 1
    for debugitem in xmlitem:
        fulldebugarray.append({'name':debugitem.attrib['name'],'longname':debugitem.attrib['longname'],'id':debugnum})
        debugnum = debugnum + 1
    return fulldebugarray

def decode_variables(xmlitem):
    fullvardict = {}
    category_id = 1
    for category in xmlitem:
        newcategoryarray = []
        if(category.tag != 'category'):
            raise ValueError('Incorrect XML tag. Should be "category", was "{}"'.format(category.tag))
        else:
            var_id = 1
            for var in category:
                if(var.tag != 'var'):
                    raise ValueError('Incorrect XML tag. Should be "var", was "{}"'.format(var.tag))
                else:
                    var_fmt = var.attrib['format']
                    if(var_fmt != 'f32' and var_fmt != 'i8' and var_fmt != 'i16' and var_fmt != 'i32'):
                        raise ValueError('Incorrect format. Recognizes "f32", "i8", "i16", or "i32". Found "{}"'.format(var_fmt))
                    else:
                        newvardict = {'name':var.attrib['name'],
                                    'format':var.attrib['format'],
                                    'default':var.attrib['default'],
                                    'longname':var.attrib['longname'],
                                    'description':var.attrib['description'],
                                    'min':var.attrib['min'],
                                    'max':var.attrib['max'],
                                    'id':(category_id<<8)+var_id}
                        newcategoryarray.append(newvardict)
                        var_id = var_id + 1
            fullvardict[category.attrib['name']] = newcategoryarray
            category_id = category_id + 1
    return fullvardict

def generate_c_header_vars(fullvars):
    with open('project_configuration_variables.h','w') as f:
        f.write('''
/******************************************************************************
 * Filename: project_configuration_variables.h
 ******************************************************************************

 Copyright (c) {} David Miller

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
 '''.format(strftime("%Y",localtime())))

        f.write(
'''

// Auto-generated file. Do not edit!
// If changes are needed, edit the variable settings in 'datavars.xml' and
// re-run 'datavar_codegen.py' to recreate this file.

#ifndef PROJECT_CONFIGURATION_VARIABLES_H_
#define PROJECT_CONFIGURATION_VARIABLES_H_
\n\n''')

        all_categories = fullvars.keys()
        for category in all_categories:
            f.write('/***  {} Configuration Variable IDs ***/\n'.format(category))
            category_variables = fullvars[category]
            f.write('#define {:32}({})\n'.format('CONFIG_'+category+'_NUMVARS',len(category_variables)))
            f.write('#define {:32}(0x{:04X})\n'.format('CONFIG_'+category+'_PREFIX',category_variables[0]['id'] & 0xFF00))
            for each_var in category_variables:
                f.write('#define {:32}(0x{:04X}) //{}:{}\n'.format('CONFIG_'+category+'_'+each_var['name'], each_var['id'],each_var['format'],each_var['description']))
            f.write('/***  {} Default Values ***/\n'.format(category))
            for each_var in category_variables:
                if(each_var['format'] == 'f32'):
                    f.write('#define {:32}({}f)\n'.format('DFLT_'+category+'_'+each_var['name'], float(each_var['default'])))
                else:
                    f.write('#define {:32}({})\n'.format('DFLT_'+category+'_'+each_var['name'], int(each_var['default'])))
            f.write('/***  {} Minimum Values ***/\n'.format(category))
            for each_var in category_variables:
                if(each_var['format'] == 'f32'):
                    f.write('#define {:32}({}f)\n'.format('MIN_'+category+'_'+each_var['name'], float(each_var['min'])))
                else:
                    f.write('#define {:32}({})\n'.format('MIN_'+category+'_'+each_var['name'], int(each_var['min'])))
                
            f.write('/***  {} Maximum Values ***/\n'.format(category))
            for each_var in category_variables:
                if(each_var['format'] == 'f32'):
                    f.write('#define {:32}({}f)\n'.format('MIN_'+category+'_'+each_var['name'], float(each_var['max'])))
                else:
                    f.write('#define {:32}({})\n'.format('MIN_'+category+'_'+each_var['name'], int(each_var['max'])))
            f.write('\n\n')
        f.write('#endif // PROJECT_CONFIGURATION_VARIABLES_H_\n')

def generate_c_header_debugs(all_debugs):
    with open('project_live_data_ids.h','w') as f:
        f.write('''
/******************************************************************************
 * Filename: project_live_data_ids.h
 ******************************************************************************

 Copyright (c) {} David Miller

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
 '''.format(strftime("%Y",localtime())))

        f.write(
'''

// Auto-generated file. Do not edit!
// If changes are needed, edit the variable settings in 'datavars.xml' and
// re-run 'datavar_codegen.py' to recreate this file.

#ifndef PROJECT_LIVE_DATA_IDS_H_
#define PROJECT_LIVE_DATA_IDS_H_
\n\n''')

        f.write('// Debugging output id definitions\n')
        f.write('#define {:32}({})\n'.format('MAX_LIVE_DATA_CHOICES',len(all_debugs)))
        f.write('#define {:32}(0)\n'.format('LIVE_CHOICE_UNUSED',len(all_debugs)))
        for debug in all_debugs:
            f.write('#define {:32}({}) // {}\n'.format('LIVE_CHOICE_'+debug['name'],debug['id'],debug['longname']))

        f.write('\n#endif // PROJECT_LIVE_DATA_IDS_H_\n')

def generate_python_header_variables(fullvars):
    with open('config_vars.py','w') as f:
        f.write(
'''
#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Auto-generated file. Do not edit!
If changes are needed, edit the variable settings in 'datavars.xml' and
re-run 'datavar_codegen.py' to recreate this file.

Defines the configuration variables in the motor controller
"""

from collections import namedtuple

Config_Var = namedtuple('Config_Var', ['name','id','format','longname','description','default','min','max'])

MCU_Config_Vars = {
''')
        all_categories = fullvars.keys()
        for category in all_categories:
            f.write("\t'{}':[\n".format(category))
            category_variables = fullvars[category]
            for each_var in category_variables:
                if(each_var['format']=='f32'):
                    default_val=float(each_var['default'])
                    min_val=float(each_var['min'])
                    max_val=float(each_var['max'])
                else:
                    default_val=int(each_var['default'])
                    min_val=int(each_var['min'])
                    max_val=int(each_var['max'])
                f.write("""
            Config_Var(name='{}',id=0x{:04X},format='{}',
            default={},min={},max={},
            longname='{}',
            description='{}'),
""".format(each_var['name'],each_var['id'],each_var['format'],
                default_val, min_val, max_val,
                each_var['longname'],each_var['description']))
            f.write('\t],\n')
        f.write('}\n')
def generate_python_header_debugs(all_debug):
    with open('live_data_ids.py','w') as f:
        f.write(
'''
#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Auto-generated file. Do not edit!
If changes are needed, edit the variable settings in 'datavars.xml' and
re-run 'datavar_codegen.py' to recreate this file.

Defines the ID codes for debugging outputs.
"""

class Debug_IDs:
    variables = [
'''
        )
        for debug in all_debugs:
            f.write("\t\t{{'name':'{}', 'id': {}, 'longname': '{}'}},\n".format(debug['name'],debug['id'],debug['longname']))
        f.write('\t]\n\n')
if __name__ == '__main__':
    for item in var_config:
        if(item.tag == 'variables'):
            all_variables = decode_variables(item)
            generate_c_header_vars(all_variables)
            generate_python_header_variables(all_variables)
        if(item.tag == 'debugoutputs'):
            all_debugs = decode_debugoutputs(item)
            generate_c_header_debugs(all_debugs)
            generate_python_header_debugs(all_debugs)
