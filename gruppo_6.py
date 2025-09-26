import sys
import Sofa

#definizione costanti
Straight_length_1 = 115
Straight_length_2 = 140
Straight_length_3 = 175
Curved_length_1 = 40
Curved_length_2 = 50
Curved_length_3 = 65
Tube_radius_1 = 1
Tube_radius_2 = 0.85
Tube_radius_3 = 0.7
Radius_curvature_1 = 105
Radius_curvature_2 = 115
Radius_curvature_3 = 135

def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='BeamAdapter Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant Sofa.Component.Topology.Container.Grid Sofa.Component.Visual Sofa.Component.Mapping.Linear Sofa.Component.Topology.Container.Dynamic Sofa.GL.Component.Rendering3D')
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields')
    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    # TUBO 1

    TUBE_1 = rootNode.addChild('TUBE_1', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_1.addObject('RodStraightSection', name='Straight_section', length = Straight_length_1, radius = Tube_radius_1, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('RodSpireSection', name='Curved_section', length = Curved_length_1, spireDiameter = 2*Radius_curvature_1, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('WireRestShape', template='Rigid3d', name='RestShape_1', wireMaterials='@Straight_section @Curved_section')

    #proprieta meccaniche

    TUBE_1.addObject('EdgeSetTopologyContainer', name='meshLines_1')
    TUBE_1.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_1.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_1.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_1')

    # TUBO 2

    TUBE_2 = rootNode.addChild('TUBE_2', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_2.addObject('RodStraightSection', name='Straight_section', length = Straight_length_2, radius = Tube_radius_2, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('RodSpireSection', name='Curved_section', length = Curved_length_2, spireDiameter = 2*Radius_curvature_2, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('WireRestShape', template='Rigid3d', name='RestShape_2', wireMaterials='@Straight_section @Curved_section')

    #proprieta meccaniche

    TUBE_2.addObject('EdgeSetTopologyContainer', name='meshLines_2')
    TUBE_2.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_2.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_2.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_2')

    # TUBO 3

    TUBE_3 = rootNode.addChild('TUBE_3', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_3.addObject('RodStraightSection', name='Straight_section', length = Straight_length_3, radius = Tube_radius_3, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('RodSpireSection', name='Curved_section', length = Curved_length_3, spireDiameter = 2*Radius_curvature_3, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('WireRestShape', template='Rigid3d', name='RestShape_3', wireMaterials='@Straight_section @Curved_section')

    #proprieta meccaniche

    TUBE_3.addObject('EdgeSetTopologyContainer', name='meshLines_3')
    TUBE_3.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_3.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_3.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_3')

    #strumento complessivo

    CTR = rootNode.addChild('InstrumentCombined')
    CTR.addObject('EulerImplicitSolver', rayleighStiffness='0.2', rayleighMass='0.1', printLog='false')
    CTR.addObject('BTDLinearSolver')
    CTR.addObject('RegularGridTopology', name='meshLinesCombined', nx='181', ny='1', nz='1', xmin='0.0', xmax='1.0', ymin='0', ymax='0', zmin='1', zmax='1')
    CTR.addObject('MechanicalObject', template='Rigid3d', name='DOFs', showIndices='0', ry='-90')
    CTR.addObject('WireBeamInterpolation', name="Interpol_1", WireRestShape='@../RestShape_1')
   # CTR.addObject('AdaptiveBeamForceFieldAndMass', name="Tube1ForceField", interpolation="@InterpolTube1")
    CTR.addObject('WireBeamInterpolation', name="Interpol_2", WireRestShape='@../RestShape_2')
   # CTR.addObject('AdaptiveBeamForceFieldAndMass', name="Tube2ForceField", interpolation="@InterpolTube2")
    CTR.addObject('WireBeamInterpolation', name="Interpol_3", WireRestShape='@../RestShape_3')
   # CTR.addObject('AdaptiveBeamForceFieldAndMass', name="Tube3ForceField", interpolation="@InterpolTube3")
    CTR.addObject('FixedProjectiveConstraint', name='FixedConstraint', indices='0')
    CTR.addObject('LinearSolverConstraintCorrection', wire_optimization='true')

    #collisioni del CTR

    

    #proprieta visuali 1

    visu_1 = CTR.addChild('visu_1', activated='true')
    visu_1.addObject('MechanicalObject', name='Quads')
    visu_1.addObject('QuadSetTopologyContainer', name='Container_1')
    visu_1.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_1.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_1.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='2', input='@../meshLinesCath', output='@ContainerCath', flipNormals='true')
    visu_1.addObject('AdaptiveBeamMapping',  name='VisuMap_1', useCurvAbs='1', printLog='0', interpolation='@../Interpol_1', input='@../DOFs', output='@Quads')
    visu_1.addObject('OglModel', name='Visual', color='1 0 0 1', quads="@../Container_1.quads")
    visu_1.addObject('IdentityMapping', name='VisuMap_1', useCurvAbs="1", interpolation="@../Interpol_1", input="@../DOFs", output="@Quads")





    return 0;


def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()