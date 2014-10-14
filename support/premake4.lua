-- HACK: solves a bug with generating VS2013 projects under premake4.4-beta5
vstudio = premake.vstudio

-- Create standalone solution
if EVDS_STANDALONE ~= false then
  SIMC_STANDALONE = false
  solution "evds"
     dofile("./../external/simc/support/premake4_common.lua")
     dofile("./../external/simc/support/premake4.lua")
end


--------------------------------------------------------------------------------
-- EVDS documentation
--------------------------------------------------------------------------------
newaction {
   trigger     = "evdsdoc",
   description = "Generate documentation for EVDS",
   execute     = function()
     if EVDS_STANDALONE ~= false
     then os.chdir("../doc")
     else os.chdir("./../external/evds/doc")
     end
     os.execute("doxygen")
   end
}
if _ACTION == "evdsdoc" then return end


--------------------------------------------------------------------------------
-- Generate EVDS default materials include
--------------------------------------------------------------------------------
local evds_defmat = "../source/core/evds_database.inc"
local evds_matxml = "../source/core/evds_database.xml"
if true then -- (not os.isfile(evds_defmat))
  -- Pad XML with C strings
  local xml_contents = ""
  for line in io.lines(evds_matxml) do
    local line = string.gsub(string.gsub(line,"\r",""),"\t","")
    xml_contents = xml_contents.."\t\""..string.gsub(line,"\"","'").."\\n\"\n"
  end
  
  -- Generate source code
  local source = "char* EVDS_Internal_Database = \n"..xml_contents..";\n"

  -- Save source code
  local file = io.open(evds_defmat,"wb+")
  file:write(source)
  file:close()
end

newaction {
   trigger     = "evdsdb",
   description = "Generate database lookup",
   execute     = function () end
}
if _ACTION == "evdsdb" then return end


--------------------------------------------------------------------------------
-- External Vessel Dynamics Simulator
--------------------------------------------------------------------------------
project "evds"
   uuid "EE4E6F16-F056-1746-B855-ADA3599F7386"
   library()
   language "C"
   includedirs { "../include",
                 "../external/simc/include" }
   files { "../source/core/**",
           "../source/objects/**",
           "../source/propagators/**",
           "../source/models/**",
           "../include/**" }
   defines { "EVDS_LIBRARY", "SIMC_LIBRARY" }
   configuration { "*Dynamic*" }
      links { "simc" }


--------------------------------------------------------------------------------
-- EVDS OpenGL Renderer
--------------------------------------------------------------------------------
--project "evds_opengl"
--   uuid "53255589-3143-4E71-864E-967A6DCE1F7C"
--   library()
--   language "C"
--   includedirs { "../include",
--                 "../external/simc/include" }
--   files { "../source/evds_opengl/**",
--           "../include/**" }
--   defines { "EVDS_LIBRARY", "SIMC_LIBRARY" }


--------------------------------------------------------------------------------
-- Tutorials
--------------------------------------------------------------------------------
if EVDS_STANDALONE ~= false then
   function tutorial(index)
      project("evds_tutorial"..index)
         uuid("740FC406-98AF-B54D-B226-CAF40650FF"..index.."E")
         kind "ConsoleApp"
         language "C"
         includedirs { "../include",
                       "../external/simc/include",
                       "../tutorials" }
         files { "../tutorials/evds_tutorial"..index..".c" }
         links { "evds", "simc" }
         
         if index == 3 then
            links { "glfw "}
            configuration { "windows" }
               links { "opengl32" }
         end
   end
   
   -- Add tutorials
   tutorial(1)
   tutorial(2)
   
   -- Add unit tests
   project "evds_tests"
      uuid "28E158E3-1600-4B73-A662-8E9228D33E12"
      kind "ConsoleApp"
      language "C"
      includedirs { "../include",
                    "../external/simc/include",
                    "../tests" }
      files { "../tests/**" }
      links { "evds", "simc" }
end
