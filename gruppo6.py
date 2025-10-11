import sys
import os
import Sofa
import Sofa.Core
import Sofa.Simulation
import csv
import numpy as np

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
        
        self.print_interval = 1
        self.last_print_time = 0.0
        
	
    #IMPLEMENTAZIONE CONTROLLORE DA TASTIERA	
	
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
            self.translate(self.tube, -1.0)		# le traslazioni (positive o negative), sono nell'ordine di 1mm
        elif key == "K":
            print("Scelta traslazione in avanti")
            self.translate(self.tube, 1.0)
        elif key == "M":
            print("Scelta rotazione oraria")
            self.rotate(self.tube, -0.4)		# le rotazioni (positive o negative), sono pari a 0.4 radianti, ossia circa 22.5 gradi
        elif key == "N":
            print("Scelta rotazione antioraria")
            self.rotate(self.tube, 0.4)
        elif key == "W":
            print("Generazione workspace")
            self.generate_workspace("work_space_1.txt")


    #IMPLEMENTAZIONE FUNZIONI RICHIAMATE DAL CONTROLLORE

    def translate(self, tube, quantity):
        with self.ir_controller.xtip.writeable() as d: d[tube-1] = d[tube-1] + quantity
    def rotate(self, tube, quantity):
        with self.ir_controller.rotationInstrument.writeable() as d: d[tube-1] = d[tube-1] + quantity

    def generate_workspace(self, filename):
        workspace_points = []

        insertion_range_1 = range(0, Straight_length_1 + Curved_length_1, 40)
        rotation_range_1  = range(-90, 91, 45)
        insertion_range_2 = range(0, Straight_length_2 + Curved_length_2, 40)
        rotation_range_2  = range(-90, 91, 45)
        insertion_range_3 = range(0, Straight_length_3 + Curved_length_3, 40)	
        rotation_range_3  = range(-90, 91, 45)

        i=1

        n_ins1 = len(insertion_range_1)
        n_rot1 = len(rotation_range_1)
        n_ins2 = len(insertion_range_2)
        n_rot2 = len(rotation_range_2)
        n_ins3 = len(insertion_range_3)
        n_rot3 = len(rotation_range_3)

        total_points = n_ins1 * n_rot1 * n_ins2 * n_rot2 * n_ins3 * n_rot3	# calcola il numero di punti che formeranno il workspace (per dare una misura del tempo necessario)

        for ins1 in insertion_range_1:
            for rot1 in rotation_range_1:
                for ins2 in insertion_range_2:
                    for rot2 in rotation_range_2:
                        for ins3 in insertion_range_3:
                            for rot3 in rotation_range_3:			# ossia, fai compiere a tutti i giunti tutti i movimenti possibili

                                print("Generazione punto",i,"/",total_points, end="\r")
                            
                                with self.ir_controller.xtip.writeable() as d:
                                    d[0] = ins1
                                    d[1] = ins2
                                    d[2] = ins3
                                with self.ir_controller.rotationInstrument.writeable() as d:
                                    d[0] = rot1
                                    d[1] = rot2
                                    d[2] = rot3
                                i=i+1
                           
                                Sofa.Simulation.animate(self.rootNode, self.rootNode.dt.value)	# procedi con un passo di simulazione

                            
                                tip_pose = self.rootNode.CTR.DOFs.position.value[-1]
                                tip_position = tip_pose[:3]

                            
                                workspace_points.append([
                                    ins1, rot1,
                                    ins2, rot2,
                                    ins3, rot3,
                                    tip_position[0], tip_position[1], tip_position[2]		# aggiungi il punto i-esimo alla lista
                                ])

        with open(filename, "w", newline="") as f:						# salva su file
            writer = csv.writer(f, delimiter='\t')
            writer.writerow([
                "Ins1", "Rot1", "Ins2", "Rot2", "Ins3", "Rot3", "X", "Y", "Z"
            ])
            writer.writerows(workspace_points)

        print(f"Workspace salvato in {filename}")
        
   
        
    def onAnimateEndEvent(self, event):
         self.register_force_value()
        
    def register_force_value(self):
        current_time = self.rootNode.time.value    
        solver = self.rootNode.getObject("solver")
        forces_vector = solver.constraintForces.value
        if forces_vector is not None and forces_vector.size > 0 and current_time - self.last_print_time > self.print_interval:
            force_value = np.linalg.norm(forces_vector)
            print(f"-------FORZA RILEVATA------")
            print(f"    Intensità della forza pari a: {force_value:.6f} N")
            print("--------------------------\n")
            self.last_print_time = current_time


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='Sofa.Component.AnimationLoop Sofa.Component.SolidMechanics.Spring MultiThreading Sofa.Component.Topology.Mapping Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Collision.Response.Contact Sofa.Component.Collision.Geometry Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Detection.Algorithm BeamAdapter Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant Sofa.Component.Topology.Container.Grid Sofa.Component.Visual Sofa.Component.Mapping.Linear Sofa.Component.Topology.Container.Dynamic Sofa.GL.Component.Rendering3D CGALPlugin Sofa.Component.IO.Mesh Sofa.Component.Engine.Select Sofa.Component.Mass')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.gravity = [0, 0, 0]
    rootNode.dt = 0.05

    # import mesh superficiale

    def get_mesh_path():
    	return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mesh')		# definizione percorso mesh
    
    heart = rootNode.addChild('Heart')
    heart.addObject('EulerImplicitSolver', rayleighStiffness="0.1", rayleighMass="0.1")	# import solver
    heart.addObject('SparseLDLSolver')
    meshpath = get_mesh_path()
    heart.addObject('MeshOBJLoader', name="heartLoader", filename=os.path.join(meshpath, 'heart.obj'), translation="0 -50 10", rotation="-45 180 0", scale="10")	# caricamento mesh di partenza
    heart.addObject('MeshGenerationFromPolyhedron', name="tetraGenerator", inputPoints="@heartLoader.position", inputTriangles="@heartLoader.triangles", inputQuads="@heartLoader.quads", drawTetras="0", facetSize="5", facetApproximation="1", cellRatio="2", cellSize="5")	# creazione mesh volumetrica
    heart.addObject('MechanicalObject', name="dofs", position="@tetraGenerator.outputPoints")	# geometria della mesh volumetrica
    heart.addObject('TetrahedronSetTopologyContainer', name="topo", tetrahedra="@tetraGenerator.outputTetras")
    heart.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    heart.addObject('ParallelTetrahedronFEMForceField', name="FEM", youngModulus="60e6", poissonRatio="0.45", method="large") # proprietà fisiche
    heart.addObject('UniformMass', name="mass", totalMass="100.0")
    heart.addObject('LinearSolverConstraintCorrection')
    heartVisu = heart.addChild('Visu')											      # proprietà visuali
    heartVisu.addObject('OglModel', name="Visual", src="@../heartLoader", color="0.7 0.3 0.1")
    heartVisu.addObject('BarycentricMapping', input="@..", output="@Visual")
    heartCollis = heart.addChild('Collision')										      # collisioni
    heartCollis.addObject('MeshTopology', src="@../heartLoader")
    heartCollis.addObject('MechanicalObject')
    heartCollis.addObject('TriangleCollisionModel')
    heartCollis.addObject('LineCollisionModel')
    heartCollis.addObject('PointCollisionModel')
    heartCollis.addObject('BarycentricMapping', input="@..", output="@.")
    constraintNode = heart.addChild('Constraints')									      # vincoli di posizione per il cuore
    constraintNode.addObject('BoxROI', name="fixedBox", box="10 -1 0 50 60 90", drawBoxes="1", position="@../dofs.rest_position")
    constraintNode.addObject('FixedProjectiveConstraint', indices="@fixedBox.indices")


    # collision pipeline

    solver = rootNode.addObject('GenericConstraintSolver', name='solver', computeConstraintForces="1", tolerance='1e-6', maxIterations='1000')
    rootNode.addObject('CollisionPipeline', verbose='0')
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('CollisionResponse', name='response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', name='proximity', alarmDistance='2', contactDistance='0.5', angleCone='0.0')

    #TUBO 1

    TUBE_1 = rootNode.addChild('TUBE_1', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_1.addObject('RodStraightSection', name='StraightSection', length = Straight_length_1, radius = Tube_radius_1, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('RodSpireSection', name='SpireSection', length = Curved_length_1, spireDiameter = 2*Radius_curvature_1, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_1.addObject('WireRestShape', template='Rigid3d', name='RestShape_1', wireMaterials='@StraightSection @SpireSection')

    #   -proprietà meccaniche

    TUBE_1.addObject('EdgeSetTopologyContainer', name='meshLines_1')
    TUBE_1.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_1.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_1.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_1')

    #TUBO 2

    TUBE_2 = rootNode.addChild('TUBE_2', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_2.addObject('RodStraightSection', name='StraightSection', length = Straight_length_2, radius = Tube_radius_2, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('RodSpireSection', name='SpireSection', length = Curved_length_2, spireDiameter = 2*Radius_curvature_2, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_2.addObject('WireRestShape', template='Rigid3d', name='RestShape_2', wireMaterials='@StraightSection @SpireSection')

    #   -proprietà meccaniche

    TUBE_2.addObject('EdgeSetTopologyContainer', name='meshLines_2')
    TUBE_2.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_2.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_2.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_2')

    #TUBO 3

    TUBE_3 = rootNode.addChild('TUBE_3', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_3.addObject('RodStraightSection', name='StraightSection', length = Straight_length_3, radius = Tube_radius_3, youngModulus=1e4, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('RodSpireSection', name='SpireSection', length = Curved_length_3, spireDiameter = 2*Radius_curvature_3, spireHeight=0.0, youngModulus=1e4, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80)
    TUBE_3.addObject('WireRestShape', template='Rigid3d', name='RestShape_3', wireMaterials='@StraightSection @SpireSection')

    #   -proprietà meccaniche

    TUBE_3.addObject('EdgeSetTopologyContainer', name='meshLines_3')
    TUBE_3.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_3.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_3.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_3')

    #STRUMENTO ASSEMBLATO

    CTR = rootNode.addChild('CTR')
    CTR.addObject('EulerImplicitSolver', rayleighStiffness='0.2', rayleighMass='0.1', printLog='false')
    CTR.addObject('BTDLinearSolver')
    CTR.addObject('RegularGridTopology', name='meshLinesCombined', nx='241', ny='1', nz='1', xmin='0.0', xmax='1.0', ymin='0', ymax='0', zmin='1', zmax='1')
    CTR.addObject('MechanicalObject', template='Rigid3d', name='DOFs', showIndices='0', ry='-90')
    CTR.addObject('WireBeamInterpolation', name='Interpol_1', WireRestShape='@../TUBE_1/RestShape_1')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube1ForceField', interpolation='@Interpol_1')
    CTR.addObject('WireBeamInterpolation', name='Interpol_2', WireRestShape='@../TUBE_2/RestShape_2')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube2ForceField', interpolation='@Interpol_2')
    CTR.addObject('WireBeamInterpolation', name='Interpol_3', WireRestShape='@../TUBE_3/RestShape_3')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube3ForceField', interpolation='@Interpol_3')
    CTR.addObject('InterventionalRadiologyController', template="Rigid3d", name="IRController", instruments="Interpol_1 Interpol_2 Interpol_3", xtip="1 0 0", step="3", rotationInstrument="0 0 0", controlledInstrument="0", startingPos="0 0 0 0 0 0 0.")
    CTR.addObject('RestShapeSpringsForceField', points="@IRController.indexFirstNode", stiffness="1e8", angularStiffness="1e8")
    CTR.addObject('FixedProjectiveConstraint', name='FixedConstraint', indices='0')
    CTR.addObject('LinearSolverConstraintCorrection', wire_optimization='true')

    #   -collisioni

    collis = CTR.addChild('Collis')
    collis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    collis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    collis.addObject('MechanicalObject', name='MechanicalObject', template='Vec3d')
    collis.addObject('MultiAdaptiveBeamMapping', controller='../IRController')
    collis.addObject('LineCollisionModel')
    collis.addObject('PointCollisionModel')

    #   -proprietà visuali 1

    visu_1 = CTR.addChild('visu_1', activated='true')
    visu_1.addObject('MechanicalObject', name='Quads')
    visu_1.addObject('QuadSetTopologyContainer', name='Container_1')
    visu_1.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_1.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_1.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius=Tube_radius_1, input='@../../TUBE_1/meshLines_1', output='@Container_1', flipNormals='true')
    visu_1.addObject('AdaptiveBeamMapping',  name='VisuMap_1', useCurvAbs='1', printLog='0', interpolation='@../Interpol_1', input='@../DOFs', output='@Quads')
    visuOgl_1 = visu_1.addChild('visuOgl_1', activated='true')
    visuOgl_1.addObject('OglModel', name='Visual', color='0.75 0.75 0.75 1', quads="@../Container_1.quads")
    visuOgl_1.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #   -proprietà visuali 2

    visu_2 = CTR.addChild('visu_2', activated='true')
    visu_2.addObject('MechanicalObject', name='Quads')
    visu_2.addObject('QuadSetTopologyContainer', name='Container_2')
    visu_2.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_2.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_2.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius=Tube_radius_2, input='@../../TUBE_2/meshLines_2', output='@Container_2', flipNormals='true')
    visu_2.addObject('AdaptiveBeamMapping',  name='VisuMap_2', useCurvAbs='1', printLog='0', interpolation='@../Interpol_2', input='@../DOFs', output='@Quads')
    visuOgl_2 = visu_2.addChild('visuOgl_2', activated='true')
    visuOgl_2.addObject('OglModel', name='Visual', color='1 0 0 1', quads="@../Container_2.quads")
    visuOgl_2.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #   -proprietà visuali 3

    visu_3 = CTR.addChild('visu_3', activated='true')
    visu_3.addObject('MechanicalObject', name='Quads')
    visu_3.addObject('QuadSetTopologyContainer', name='Container_3')
    visu_3.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_3.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_3.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius=Tube_radius_3, input='@../../TUBE_3/meshLines_3', output='@Container_3', flipNormals='true')
    visu_3.addObject('AdaptiveBeamMapping',  name='VisuMap_3', useCurvAbs='1', printLog='0', interpolation='@../Interpol_3', input='@../DOFs', output='@Quads')
    visuOgl_3 = visu_3.addChild('visuOgl_3', activated='true')
    visuOgl_3.addObject('OglModel', name='Visual', color='0 0 1 1', quads="@../Container_3.quads")
    visuOgl_3.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    # aggiunta controllore alla scena

    rootNode.addObject(KeyBoardController(
        name="KeyBoardController",
        irController=CTR.getObject('IRController'),
        rootNode=rootNode
    ))
    
    print("")
    print("-------------------------------------------------------------------------")
    print("Keyboard Controller Attivo")
    print("Selezionare il tubo da controllare (Ctrl + 1/2/3)")
    print("Successivamente selezionare il tipo di moto:")
    print("traslazione in avanti o indietro (Ctrl + k/j)")
    print("rotazione in verso orario o antiorario (Ctrl + m/n)")
    print("Il workspace del tubo 3 può essere generato e salvato su file (Ctrl + w) ")
    print("-------------------------------------------------------------------------")
    print("")

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
