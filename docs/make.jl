using Documenter,RobotOSDocs,RobotOS
makedocs(modules=[RobotOS],
        doctest=false, clean=true,
        format =:html,
        authors="Josh Langsfeld, Huckleberry Febbo",
        sitename="RobotOS.jl",
        pages = Any[
        "Home" => "index.md",
        "Tutorials"=>Any[
              "tutorials/Gazebo/main.md"
               ]
               ]
               )

deploydocs(
    deps=Deps.pip("mkdocs","python-markdown-math"),
    repo="github.com/huckl3b3rry87/RobotOSDocs.jl",
    target="build",
    osname="linux",
    julia="0.6",
    make=nothing)
