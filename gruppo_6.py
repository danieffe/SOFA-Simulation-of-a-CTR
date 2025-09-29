import sys
import Sofa
import Sofa.Core

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

class KeyBoardController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.name = 'KeyBoardController'
        self.ir_controller = kwargs.get('irController')
        self.rootNode = kwargs.get('rootNode')
        self.tube = 1    # ( 1 = TUBE_1, 2 = TUBE_2, 3 = TUBE_3 )
        self.action = 'k' 

        #definizione del csv

        #

        #fine csv

        print("BENVENUTO")
        print("Selezionare il tubo da controllare (Ctrl + 1/2/3)")
        print("Successivamente selezionare il tipo di moto: traslazione in avanti o indietro (Ctrl + k/j) - rotazione in verso orario o antiorario (Ctrl + m/n)")

    def onKeypressedEvent(self, c):
        key = c['key']
        if key == "1":
            print("Selezionato il tubo 1")
            self.tube = 1
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == "2":
            print("Selezionato il tubo 2")
            self.tube = 2
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == "3":
            print("Selezionato il tubo 3")
            self.tube = 3
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == "J":
            print("Scelta traslazione all'indietro")
            self.action = 'j'
            self.translate(self.tube, -1.0)
        elif key == "K":
            print("Scelta traslazione in avanti")
            self.action = 'k'
            self.translate(self.tube, 1.0)
        elif key == "M":
            print("Scelta rotazione oraria")
            self.action = 'm'
            self.rotate(self.tube, -1.0)
        elif key == "N":
            print("Scelta rotazione antioraria")
            self.action = 'n'
            self.rotate(self.tube, 1.0)

    #IMPLEMENTAZIONE CONTROLLO VERO E PROPRIO

    def translate(self, tube, quantity):
        with self.ir_controller.xtip.writeable() as d: d[tube-1] = d[tube-1] + quantity
    def rotate(self, tube, quantity):
        with self.ir_controller.rotationInstrument.writeable() as d: d[tube-1] = d[tub-1]+ quantity




def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='Sofa.Component.AnimationLoop Sofa.Component.SolidMechanics.Spring MultiThreading Sofa.Component.Topology.Mapping Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Collision.Response.Contact Sofa.Component.Collision.Geometry Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Detection.Algorithm BeamAdapter Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant Sofa.Component.Topology.Container.Grid Sofa.Component.Visual Sofa.Component.Mapping.Linear Sofa.Component.Topology.Container.Dynamic Sofa.GL.Component.Rendering3D')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.gravity = [0, 0, 0]
    rootNode.dt = 0.05

    #collision pipeline

    solver = rootNode.addObject('GenericConstraintSolver', name='solver', computeConstraintForces="1", tolerance='1e-6', maxIterations='1000')
    rootNode.addObject('CollisionPipeline', verbose='0')
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('CollisionResponse', name='response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', name='proximity', alarmDistance='2', contactDistance='0.5', angleCone='0.0')

    # TUBO 1

    TUBE_1 = rootNode.addChild('TUBE_1', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_1.addObject('RodStraightSection', name='StraightSection', length = Straight_length_1, radius = Tube_radius_1, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('RodSpireSection', name='SpireSection', length = Curved_length_1, spireDiameter = 2*Radius_curvature_1, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('WireRestShape', template='Rigid3d', name='RestShape_1', wireMaterials='@StraightSection @SpireSection')

    #proprieta meccaniche

    TUBE_1.addObject('EdgeSetTopologyContainer', name='meshLines_1')
    TUBE_1.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_1.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_1.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_1')

    # TUBO 2

    TUBE_2 = rootNode.addChild('TUBE_2', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_2.addObject('RodStraightSection', name='StraightSection', length = Straight_length_2, radius = Tube_radius_2, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('RodSpireSection', name='SpireSection', length = Curved_length_2, spireDiameter = 2*Radius_curvature_2, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('WireRestShape', template='Rigid3d', name='RestShape_2', wireMaterials='@StraightSection @SpireSection')

    #proprieta meccaniche

    TUBE_2.addObject('EdgeSetTopologyContainer', name='meshLines_2')
    TUBE_2.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_2.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_2.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_2')

    # TUBO 3

    TUBE_3 = rootNode.addChild('TUBE_3', bbox='-3 -6 -3 3 3 3')

    #sezioni e unione

    TUBE_3.addObject('RodStraightSection', name='StraightSection', length = Straight_length_3, radius = Tube_radius_3, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('RodSpireSection', name='SpireSection', length = Curved_length_3, spireDiameter = 2*Radius_curvature_3, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('WireRestShape', template='Rigid3d', name='RestShape_3', wireMaterials='@StraightSection @SpireSection')

    #proprieta meccaniche

    TUBE_3.addObject('EdgeSetTopologyContainer', name='meshLines_3')
    TUBE_3.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_3.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_3.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_3')

    #strumento complessivo

    CTR = rootNode.addChild('CTR')
    CTR.addObject('EulerImplicitSolver', rayleighStiffness='0.2', rayleighMass='0.1', printLog='false')
    CTR.addObject('BTDLinearSolver')
    CTR.addObject('RegularGridTopology', name='meshLinesCombined', nx='181', ny='1', nz='1', xmin='0.0', xmax='1.0', ymin='0', ymax='0', zmin='1', zmax='1')
    CTR.addObject('MechanicalObject', template='Rigid3d', name='DOFs', showIndices='0', ry='-90')
    CTR.addObject('WireBeamInterpolation', name='Interpol_1', WireRestShape='@../TUBE_1/RestShape_1')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube1ForceField', interpolation='@Interpol_1')
    CTR.addObject('WireBeamInterpolation', name='Interpol_2', WireRestShape='@../TUBE_2/RestShape_2')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube2ForceField', interpolation='@Interpol_2')
    CTR.addObject('WireBeamInterpolation', name='Interpol_3', WireRestShape='@../TUBE_3/RestShape_3')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube3ForceField', interpolation='@Interpol_3')
    CTR.addObject('InterventionalRadiologyController', template="Rigid3d", name="IRController", instruments="Interpol_1 Interpol_2 Interpol_3", xtip="1 0 0", step="3", rotationInstrument="0 0 0", controlledInstrument="0", startingPos="-35 58 0 0 -0.7071068 0 0.7071068")
    CTR.addObject('RestShapeSpringsForceField', points="@IRController.indexFirstNode", stiffness="1e8", angularStiffness="1e8")
    CTR.addObject('FixedProjectiveConstraint', name='FixedConstraint', indices='0')
    CTR.addObject('LinearSolverConstraintCorrection', wire_optimization='true')

    #collisioni del CTR

    collis = CTR.addChild('Collis')
    collis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    collis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    collis.addObject('MechanicalObject', name='MechanicalObject', template='Vec3d')
    #collis.addObject('MultiAdaptiveBeamMapping', controller='../IRController')
    collis.addObject('LineCollisionModel')
    collis.addObject('PointCollisionModel')

    #proprieta visuali 1

    visu_1 = CTR.addChild('visu_1', activated='true')
    visu_1.addObject('MechanicalObject', name='Quads')
    visu_1.addObject('QuadSetTopologyContainer', name='Container_1')
    visu_1.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_1.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_1.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='2', input='@../../TUBE_1/meshLines_1', output='@Container_1', flipNormals='true')
    visu_1.addObject('AdaptiveBeamMapping',  name='VisuMap_1', useCurvAbs='1', printLog='0', interpolation='@../Interpol_1', input='@../DOFs', output='@Quads')
    visuOgl_1 = visu_1.addChild('visuOgl_1', activated='true')
    visuOgl_1.addObject('OglModel', name='Visual', color='1 0 0 1', quads="@../Container_1.quads")
    visuOgl_1.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #proprieta visuali 2

    visu_2 = CTR.addChild('visu_2', activated='true')
    visu_2.addObject('MechanicalObject', name='Quads')
    visu_2.addObject('QuadSetTopologyContainer', name='Container_2')
    visu_2.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_2.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_2.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='1.5', input='@../../TUBE_2/meshLines_2', output='@Container_2', flipNormals='true')
    visu_2.addObject('AdaptiveBeamMapping',  name='VisuMap_2', useCurvAbs='1', printLog='0', interpolation='@../Interpol_2', input='@../DOFs', output='@Quads')
    visuOgl_2 = visu_2.addChild('visuOgl_2', activated='true')
    visuOgl_2.addObject('OglModel', name='Visual', color='0 1 0 1', quads="@../Container_2.quads")
    visuOgl_2.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #proprieta visuali 3

    visu_3 = CTR.addChild('visu_3', activated='true')
    visu_3.addObject('MechanicalObject', name='Quads')
    visu_3.addObject('QuadSetTopologyContainer', name='Container_3')
    visu_3.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_3.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_3.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='1', input='@../../TUBE_3/meshLines_3', output='@Container_3', flipNormals='true')
    visu_3.addObject('AdaptiveBeamMapping',  name='VisuMap_3', useCurvAbs='1', printLog='0', interpolation='@../Interpol_3', input='@../DOFs', output='@Quads')
    visuOgl_3 = visu_3.addChild('visuOgl_3', activated='true')
    visuOgl_3.addObject('OglModel', name='Visual', color='0 0 1 1', quads="@../Container_3.quads")
    visuOgl_3.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    # aggiunta controllore alla scena

    rootNode.addObject(KeyBoardController(
        name="KeyBoardController",
        irController=CTR.getObject("IRController"),
        rootNode=rootNode
    ))


    return rootNode;


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