using Documenter, AutomotiveInteraction

makedocs(
    modules = [AutomotiveInteraction],
    format = Documenter.HTML(),
    sitename="AutomotiveInteraction.jl"
    )

deploydocs(
    repo = "github.com/sisl/AutomotiveInteraction.jl.git"
    )
