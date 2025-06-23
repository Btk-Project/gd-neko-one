add_requires("gtest")

for _, file in ipairs(os.files("./**/test_*.cpp")) do
    local name = path.basename(file)
    local dir = path.directory(file)
    local conf_path = dir .. "/" .. name .. ".lua"

    -- If this file require a specific configuration, load it, and skip the auto target creation
    if os.exists(conf_path) then 
        includes(conf_path)
        goto continue
    end

    -- Otherwise, create a target for this file, in most case, it should enough
    target(name)
        add_includedirs("$(projectdir)/src")
        add_packages("godotcpp4", "gtest")
        add_files("$(projectdir)/src/**/*.cpp", file)
        set_kind("binary")
        set_default(false)
        set_encodings("utf-8")
        -- set test in different cpp versions
        local cpp_versions = {"c++20"}
        for i = 1, #cpp_versions do
            add_tests(string.gsub(cpp_versions[i], '+', 'p', 2), {group = "proto", kind = "binary", files = {file}, languages = cpp_versions[i], run_timeout = 30000})
        end
        if has_config("memcheck") then
            on_run(function (target)
                local argv = {}
                table.insert(argv, "--leak-check=full")
                table.insert(argv, target:targetfile())
                os.execv("valgrind", argv)
            end)
        end
    target_end()

    ::continue::
end