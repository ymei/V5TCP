# $Id$
#-----------------------------------------------------------
# Xilinx Planahead Tcl script to generate this project
#

#-- the GUI might already be running
#start_gui

#-- local directory:
# cd xxxx
set projDir ../[file dirname [info script]]
set projName build

#-- if the project directory exists, delete it and create a new one
if {[file exists $projDir/$projName]} {
    file delete -force $projDir/$projName
}

#-- Now create a new project
create_project $projName $projName -part xc5vlx50tff1136-1

#-- set project properties
set_property design_mode RTL [get_filesets sources_1]
set_property target_language VHDL [current_project]
set_property ng.output_hdl_format VHDL [get_filesets sim_1]

#-- add source files from the different project source directories to project
set vhdlSources [ glob src/top.vhd \
                      ipcore_dir/clock_generator.vhd ]

# `import_files' copies files into the project, while `add_files' doesn't copy
# import_files -fileset [get_filesets sources_1] -force -norecurse -flat $vhdlSources
add_files -fileset [get_filesets sources_1] -norecurse $vhdlSources

#-- add constraints file to project
add_files -fileset [get_filesets constrs_1] -norecurse src/top.ucf

#-- import IPs from sources and add to project:
set ipSources [glob ipcore_dir/cs_icon.xco \
                   ipcore_dir/cs_ila.xco \
                   ipcore_dir/cs_vio.xco ]
                   
read_ip -files $ipSources

#-- add test bench source files
# set tbSources [ glob test_bench/top_tb.vhd ]

# add_files -fileset [get_filesets sim_1] -norecurse $tbSources

#-- copy the revision file into the source directory
# file copy sources/revision.txt $projName/$projName.srcs/sources_1/imports/

#-- define the top entity
set_property top top [get_property srcset [current_run]]

#-- looks like these commands are executed automatically by the GUI
#update_compile_order -fileset sources_1
#update_compile_order -fileset sim_1

#-- set multi-processor for map and par
set_property steps.map.args.mt on [get_runs impl_1]
set_property steps.par.args.mt 4 [get_runs impl_1]

#-- these commands can be used to compile the project:
#launch_runs -runs synth_1
#wait_on_run synth_1
#launch_runs -runs impl_1
#wait_on_run impl_1
#launch_runs impl_1 -to_step Bitgen
#wait_on_run impl_1
#open_run impl_1
#open_impl_design
#close_design
