project('ds250-tool', 'c',
  version : '0.1.0',
  license : 'BSD-3-Clause',
  meson_version : '>= 0.56',
  default_options : [
    'c_std=c11',
    'warning_level=3',
    'buildtype=release',
  ],
)

add_project_arguments(
  '-Wshadow',
  '-Wstrict-prototypes',
  language : 'c',
)

link_args = []
if get_option('static')
  link_args += ['-static', '-static-libgcc']
endif

executable('ds250',
  'src/ds250.c',
  link_args : link_args,
  install : true,
)
