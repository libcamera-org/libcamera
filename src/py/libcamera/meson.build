# SPDX-License-Identifier: CC0-1.0

pycamera_sources = files([
    'py_camera_manager.cpp',
    'py_color_space.cpp',
    'py_enums.cpp',
    'py_geometry.cpp',
    'py_helpers.cpp',
    'py_main.cpp',
    'py_transform.cpp',
])

# Generate controls and properties

gen_py_controls_template = files('py_controls_generated.cpp.in')
gen_py_controls = files('gen-py-controls.py')

pycamera_sources += custom_target('py_gen_controls',
                                  input : controls_files,
                                  output : ['py_controls_generated.cpp'],
                                  command : [gen_py_controls, '--mode', 'controls', '-o', '@OUTPUT@',
                                             '-t', gen_py_controls_template, '@INPUT@'],
                                  depend_files : [py_mod_controls],
                                  env : py_build_env)

pycamera_sources += custom_target('py_gen_properties',
                                  input : properties_files,
                                  output : ['py_properties_generated.cpp'],
                                  command : [gen_py_controls, '--mode', 'properties', '-o', '@OUTPUT@',
                                             '-t', gen_py_controls_template, '@INPUT@'],
                                  depend_files : [py_mod_controls],
                                  env : py_build_env)

# Generate formats

gen_py_formats_input_files = files([
    '../../libcamera/formats.yaml',
    'py_formats_generated.cpp.in',
])

gen_py_formats = files('gen-py-formats.py')

pycamera_sources += custom_target('py_gen_formats',
                                  input : gen_py_formats_input_files,
                                  output : ['py_formats_generated.cpp'],
                                  command : [gen_py_formats, '-o', '@OUTPUT@', '@INPUT@'])

pycamera_deps = [
    libcamera_private,
    py3_dep,
    pybind11_dep,
]

pycamera_args = [
    '-fvisibility=hidden',
    '-Wno-shadow',
    '-DPYBIND11_USE_SMART_HOLDER_AS_DEFAULT',
]

destdir = get_option('libdir') / ('python' + py3_dep.version()) / 'site-packages' / 'libcamera'

pycamera = shared_module('_libcamera',
                         pycamera_sources,
                         install : true,
                         install_dir : destdir,
                         install_tag : 'python-runtime',
                         name_prefix : '',
                         dependencies : pycamera_deps,
                         cpp_args : pycamera_args)

# Create symlinks from the build dir to the source dir so that we can use the
# Python module directly from the build dir.

run_command('ln', '-fsrT', files('__init__.py'),
            meson.current_build_dir() / '__init__.py',
            check : true)

run_command('ln', '-fsrT', meson.current_source_dir() / 'utils',
            meson.current_build_dir() / 'utils',
            check : true)

install_data(['__init__.py'],
             install_dir : destdir,
             install_tag : 'python-runtime')

# \todo Generate stubs when building. See https://peps.python.org/pep-0484/#stub-files
# Note: Depends on pybind11-stubgen. To generate pylibcamera stubs:
# $ PYTHONPATH=build/src/py pybind11-stubgen --no-setup-py -o build/src/py libcamera
