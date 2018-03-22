using Documenter, AVExamples
makedocs(modules=[AVExamples],
        doctest=false, clean=true,
        format =:html,
        authors="Huckleberry Febbo",
        sitename="MAVs",
        pages = Any[
        "Home" => "index.md",
        "System Demos"=>Any[
              "demos/system/demoA.md"
              "demos/system/demoB.md"
              "demos/system/demoC.md"
              "demos/system/demoD.md"
              "demos/system/demoE.md"
              "demos/system/demoF.md"
               ],
        "Package Demos"=>Any[
             "demos/vehicle_description/demo.md"
             "demos/nloptcontrol_planner/demo.md"
             "demos/ros_chrono/demo.md"
               ],
        "MAVs.jl"=>Any[
            "mavs/index.md"
         "Miscellaneous"=>Any[
               "issues/index.md"
                ]
               ]
               )
deploydocs(
    deps=Deps.pip("mkdocs","python-markdown-math"),
    repo="github.com/JuliaMPC/AVExamples.jl.git",
    target="build",
    osname="linux",
    julia="0.6",
    make=nothing)
