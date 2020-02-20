push!(LOAD_PATH,"../src/")
using Documenter, AutomotiveInteraction

makedocs(
    modules = [AutomotiveInteraction],
    sitename="AutomotiveInteraction.jl",
    format = Documenter.HTML()
    )

#deploydocs(
#    repo = "github.com/sisl/AutomotiveInteraction.jl.git"
#    )
