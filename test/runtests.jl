using Revise
using Test
using AutomotiveInteraction
using AutomotiveDrivingModels
using AutoViz

roadway = make_roadway_interaction()
render([roadway],camera=StaticCamera(position = VecE2(1000.,1000.)))

