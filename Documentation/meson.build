# SPDX-License-Identifier: CC0-1.0

doc_install_dir = get_option('datadir') / 'doc' / 'libcamera-@0@'.format(libcamera_version)

#
# Doxygen
#

doxygen = find_program('doxygen', required : get_option('documentation'))
dot = find_program('dot', required : get_option('documentation'))

if doxygen.found() and dot.found()
    cdata = configuration_data()
    cdata.set('VERSION', 'v@0@'.format(libcamera_git_version))
    cdata.set('TOP_SRCDIR', meson.project_source_root())
    cdata.set('TOP_BUILDDIR', meson.project_build_root())
    cdata.set('OUTPUT_DIR', meson.current_build_dir())
    cdata.set('WARN_AS_ERROR', get_option('doc_werror') ? 'YES' : 'NO')

    doxygen_predefined = []
    foreach key : config_h.keys()
        doxygen_predefined += '@0@=@1@'.format(key, config_h.get(key))
    endforeach

    cdata.set('PREDEFINED', ' \\\n\t\t\t '.join(doxygen_predefined))

    doxyfile = configure_file(input : 'Doxyfile.in',
                              output : 'Doxyfile',
                              configuration : cdata)

    doxygen_input = [
        doxyfile,
        libcamera_base_headers,
        libcamera_base_public_sources,
        libcamera_base_internal_sources,
        libcamera_internal_headers,
        libcamera_ipa_headers,
        libcamera_ipa_interfaces,
        libcamera_public_headers,
        libcamera_public_sources,
        libcamera_internal_sources,
        libipa_headers,
        libipa_sources,
    ]

    if is_variable('ipu3_ipa_sources')
        doxygen_input += [ipu3_ipa_sources]
    endif

    custom_target('doxygen',
                  input : doxygen_input,
                  output : 'api-html',
                  command : [doxygen, doxyfile],
                  install : true,
                  install_dir : doc_install_dir,
                  install_tag : 'doc')
endif

#
# Sphinx
#

sphinx = find_program('sphinx-build-3', required : false)
if not sphinx.found()
    sphinx = find_program('sphinx-build', required : get_option('documentation'))
endif

if sphinx.found()
    docs_sources = [
        'camera-sensor-model.rst',
        'code-of-conduct.rst',
        'coding-style.rst',
        'conf.py',
        'contributing.rst',
        'docs.rst',
        'environment_variables.rst',
        'guides/application-developer.rst',
        'guides/introduction.rst',
        'guides/ipa.rst',
        'guides/pipeline-handler.rst',
        'guides/tracing.rst',
        'index.rst',
        'lens_driver_requirements.rst',
        'python-bindings.rst',
        'sensor_driver_requirements.rst',
        'software-isp-benchmarking.rst',
       '../README.rst',
    ]

    release = 'release=v' + libcamera_git_version

    custom_target('documentation',
                  command : [sphinx, '-D', release, '-q', '-W', '-b', 'html',
                             meson.current_source_dir(), '@OUTPUT@'],
                  input : docs_sources,
                  output : 'html',
                  build_by_default : true,
                  install : true,
                  install_dir : doc_install_dir,
                  install_tag : 'doc')

    custom_target('documentation-linkcheck',
                  command : [sphinx, '-W', '-b', 'linkcheck', meson.current_source_dir(), '@OUTPUT@'],
                  build_always_stale : true,
                  input : docs_sources,
                  output : 'linkcheck')
endif
