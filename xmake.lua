set_project("gd-neko-one")
add_rules("mode.debug", "mode.release")
set_version("0.0.1")

set_languages("c++20")

add_requires("godotcpp4")

target("gd-neko-one")
    set_kind("shared")
    add_packages("godotcpp4")
    add_files("src/*.cpp")
    after_build(function (target) 
        import("core.project.config")
        local target_file = target:targetfile();
        local gd_dep_file = path.join(os.projectdir(), "cpp-extension", "bin" , "lib" .. target:basename()
                                                                        .. "." .. config.get("plat")
                                                                        .. "." .. config.get("mode")
                                                                        .. "." .. os.arch()
                                                                        .. path.extension(target_file));
        os.cp(target_file, gd_dep_file)
        print(gd_dep_file)
    end)
