from collada import *
from CsbFile import CsbFile
from CtbFile import CtbFile
from CsbExporter import Export
from Triangle import *

import numpy as np
from math import radians
from scipy.spatial.transform import Rotation

epsilon = 1e-9

def DecomposeMatrix(matrix: np.ndarray) -> (list, list, list):
    # Extract translation
    translation = matrix[:3, 3]

    # Extract scale (length of each axis vector)
    scale = np.linalg.norm(matrix[:3, :3], axis=0)

    # Remove scale from the rotation matrix
    rotation_matrix = matrix[:3, :3] / scale

    # Convert rotation matrix to Euler angles
    # Assumes XYZ rotation order (can be adjusted as needed)
    rotation = Rotation.from_matrix(rotation_matrix)
    eulerAngles = rotation.as_euler('xyz', degrees=True)
    eulerAngles = [radians(a) for a in eulerAngles]

    return translation, eulerAngles, scale

def ImportFromDae(filePath: str, is_map_object: bool = False):
    triID = 0
    
    def getValues(materials: list):
        #try to detect the material and flags via material name
        identifiers = [None, None]
        
        if materials:
            mat = materials[0].target.name
            values = mat.split('_')
            for v in values:
                if v.startswith('MAT'):
                    MaterialAttribute = int(v.replace('MAT', ''))
                elif v.startswith('FLAG'):
                    ColFlag = int(v.replace('FLAG', ''))
                elif v.startswith('ID-A'):
                    identifiers[0] = int(v.replace('ID-A', ''))           
                elif v.startswith('ID-B'):
                    identifiers[1] = int(v.replace('ID-B', ''))
            return MaterialAttribute, ColFlag, identifiers
        else:
            return 0, 0
    
    def ImportMapObject(node: CsbFile.Node, mats: list, isSphere: bool = False):
        nonlocal csb, ID
        newColObject = csb.CollisionObject()
        
        newColObject.IsSphere = isSphere
        
        t, r, s = DecomposeMatrix(node.matrix)
        newColObject.Point1 = newColObject.Point2 = t
        
        if isSphere:
            newColObject.Radius = s[0] if s[0] > epsilon else epsilon
        else:
            newColObject.Rotation = r 
            newColObject.Size = [x if x > epsilon else epsilon for x in s]
        
        newColObject.Name = node.name.split('_')
        newColObject.Name = newColObject.Name[2:]
        newColObject.Name = '_'.join(newColObject.Name)
        
        newColObject.NodeIndex = ID - 1
        
        newColObject.Unknown = 0
        
        #print(newColObject.Name)
        #print(newColObject.Point1)
        #print()
        
        newColObject.MaterialAttribute, newColObject.ColFlag, identifiers = getValues(mats)
        newColObject.Identifier1 = identifiers[0]
        newColObject.Identifier2 = identifiers[1]
        
        csb.Objects.append(newColObject)
    
    def ImportModelSplit(node: CsbFile.Node, geometryChild: scene.GeometryNode):
        nonlocal csb, ID
        newModelSplit = csb.Model()
        
        newModelSplit.Meshes = []
        newModelSplit.Triangles = []
        newModelSplit.Positions = []
        
        t, r, s = DecomposeMatrix(node.matrix)
        newModelSplit.Translate = t
        newModelSplit.Rotation = r
        
        newModelSplit.MaterialAttribute, newModelSplit.ColFlag, _ = getValues(geometryChild.materials)
        
        newModelSplit.Name = node.name.split('_')
        newModelSplit.Name = newModelSplit.Name[1:]
        newModelSplit.Name = '_'.join(newModelSplit.Name)
        
        newModelSplit.NodeIndex = ID - 1
        
        newModelSplit.Unknown0 = 1
        newModelSplit.Unknown4 = 4
        
        newModelSplit.Positions = list(list(geometryChild.geometry.sourceById.values())[0])
        newModelSplit.Positions = [x.tolist() for x in newModelSplit.Positions]
        newModelSplit.NumVertices = len(newModelSplit.Positions)
        
        newModelSplit.Triangles = []
        
        if geometryChild.geometry.primitives and not geometryChild.geometry.primitives[0].vertex_index is None:
            normalSource = geometryChild.geometry.primitives[0].normal
            vertexSource = geometryChild.geometry.primitives[0].vertex
            for vtxIdxData, normalIdxData in zip(geometryChild.geometry.primitives[0].vertex_index, geometryChild.geometry.primitives[0].normal_index):
                vtxIdxData = vtxIdxData.tolist()
                newTri = Triangle()
                newTri.A = vtxIdxData[0]
                newTri.B = vtxIdxData[1]
                newTri.C = vtxIdxData[2]
                newTri.Vertices = [vertexSource[vtxIdxData[0]], vertexSource[vtxIdxData[1]], vertexSource[vtxIdxData[2]]]
                #print('Tri:')
                #print(newTri.Vertices)
                newTri.Normal = normalSource[normalIdxData[0]]
                newModelSplit.Triangles.append(newTri)
        
        newModelSplit.NumTriangles = len(newModelSplit.Triangles)
        
        newModelSplit.Bounding.Compute(newModelSplit.Positions)
        
        csb.Models.append(newModelSplit)
        
        
    
    def ImportNode(node: CsbFile.Node, root: bool = False):
        nonlocal csb, ID, newModel, triID
        
        if root:
            t, r, s = DecomposeMatrix(node.matrix)
            newModel.Translate = t
            newModel.Rotation = r
        
        newNode = csb.Node()
        
        newNode.ID = ID
        ID += 1
        
        newNode.Flags = 0
        
        geometryChildren = [i for i, x in enumerate(node.children) if type(x) is scene.GeometryNode]
        
        newNode.NumChildren = len(node.children) - (1 if geometryChildren else 0)
        
        csb.Nodes.append(newNode)
        
        if geometryChildren:
            geometryChild = node.children[geometryChildren[0]]
            node.children.pop(geometryChildren[0])
            
            if node.name.startswith('MAPOBJ_SPHERE'):
                ImportMapObject(node, geometryChild.materials, True)
                newNode.Flags = 2
            elif node.name.startswith('MAPOBJ_BOX'):
                ImportMapObject(node, geometryChild.materials, False)
                newNode.Flags = 3
            elif node.name.startswith('MODELSPLIT'):
                ImportModelSplit(node, geometryChild)
                newNode.Flags = 1
            else:
                newMesh = csb.Mesh()
                
                newMesh.Name = node.name
                newMesh.NodeIndex = newNode.ID
                newNode.Flags = 0
                
                newMesh.MaterialAttribute, newMesh.ColFlag, _ = getValues(geometryChild.materials)
                
                #if len(geometryChild.geometry.primitives[0]) > 0:
                    #print(list(list(geometryChild.geometry.sourceById.values())[0])[0])
                    #print(list(geometryChild.geometry.primitives[0].vertex_index))
                    #print(list(geometryChild.geometry.primitives[0].normal)[0])
                
                newMesh.Positions = list(list(geometryChild.geometry.sourceById.values())[0])
                newMesh.Positions = [x.tolist() for x in newMesh.Positions]
                newMesh.NumVertices = len(newMesh.Positions)
                
                newMesh.Triangles = []
                
                #print()
                #print(newMesh.Name)
                
                if geometryChild.geometry.primitives and not geometryChild.geometry.primitives[0].vertex_index is None:
                    normalSource = geometryChild.geometry.primitives[0].normal
                    vertexSource = geometryChild.geometry.primitives[0].vertex
                    for vtxIdxData, normalIdxData in zip(geometryChild.geometry.primitives[0].vertex_index, geometryChild.geometry.primitives[0].normal_index):
                        vtxIdxData = vtxIdxData.tolist()    
                        #print((vtxIdxData, normalIdxData))
                        newTri = Triangle()
                        newTri.A = vtxIdxData[0] + len(newModel.Positions)
                        newTri.B = vtxIdxData[1] + len(newModel.Positions)
                        newTri.C = vtxIdxData[2] + len(newModel.Positions)
                        newTri.Vertices = [vertexSource[vtxIdxData[0]], vertexSource[vtxIdxData[1]], vertexSource[vtxIdxData[2]]]
                        #print('Tri:')
                        #print(newTri.Vertices)
                        newTri.Normal = normalSource[normalIdxData[0]]
                        newTri.ID = triID
                        triID += 1
                        newMesh.Triangles.append(newTri)
                
                newMesh.NumTriangles = len(newMesh.Triangles)
            
                newModel.Meshes.append(newMesh)
                newModel.Positions.extend(newMesh.Positions)
                newModel.Triangles.extend(newMesh.Triangles)
                newModel.NumVertices += newMesh.NumVertices
                newModel.NumTriangles += newMesh.NumTriangles
                
                if len(csb.Models) == 0:
                    #No sub models so use defaults
                    csb.SubModelBounding.Min = (99999.0, 99999.0, 99999.0)
                    csb.SubModelBounding.Max = (-99999.0, -99999.0, -99999.0)
                else:
                    csb.SubModelBounding.Compute(sum([model.Positions for model in csb.Models], []))
        
        for subnode in node.children:
            ImportNode(subnode)
    
    csb = CsbFile()
    collada = Collada(filePath)
    myscene = collada.scene
    root_node_found = False
    #settings = ImportSettings().IsMapObject = is_map_object
    
    for node in myscene.nodes:
        if node.name.startswith('MODELSPLIT'):
            # model split stuff
            pass
        elif node.name == "A":
            # DEADBEEF model
            root_node_found = True
            ID = 0
            newModel = csb.Model()
            ImportNode(node, True)
            newModel.Bounding.Compute(newModel.Positions)
            csb.Models.insert(0, newModel)
    #Export(csb, f'{filePath}.re.dae')
    #for obj in csb.Objects:
    #    print(obj.Name)
    #    print(obj.Point1)
    #    print()
    assert root_node_found, "Root node not found, make sure it's named 'A'"
    return csb
            

def Import(filePath: str, name: str, is_big_endian: bool, is_map_object: bool):
    print("Loading file data")
    
    results = ImportFromDae(filePath, is_map_object)
    
    open(f'{name}_output.csb', 'wb').write(results.Write(False))
    
    #Generate a collision table
    ctbfile = CtbFile()
    ctbfile.Generate(results)
    
    open(f'{name}_output.ctb', 'wb').write(ctbfile.Write(False))
    
    