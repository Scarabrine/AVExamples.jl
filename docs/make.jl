using Documenter,RobotOSDocs,RobotOS
makedocs(modules=[RobotOS],
        doctest=false, clean=true,
        format =:html,
        authors="Huckleberry Febbo",
        sitename="ROS, Gazebo, and julia",
        pages = Any[
        "Home" => "index.md",
        "Tutorials"=>Any[
              "tutorials/Gazebo/main.md"
               ],
         "Miscellaneous"=>Any[
               "issues/index.md"
                ]
               ]
               )

deploydocs(
    deps=Deps.pip("mkdocs","python-markdown-math"),
    repo="github.com/huckl3b3rry87/RobotOSDocs.jl",
    branch = "gh-pages",
    latest = "master",
    target="build",
    osname="linux",
    julia="0.6",
    make=nothing)
