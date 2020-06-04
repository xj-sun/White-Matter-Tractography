# White-Matter-Tractography

Code here is about diffusion tensor tractography. 

A diffusion tensor image (DTI) has, as the name indicates, a tensor at each voxel The tensor is related to 
the speed of water diffusion along the various directions. Along the white matter fiber tracts, the principal
direction of diffusion is parallel to the tract (sort of similar to signal propagating along an
insulated cable), whereas outside the tracts the diffusion is less directionally restricted and thus
more isotropic. 

The code here first extract the principal eigenvector from diffusion tensor image and then Visualize
the vector image in VTK. the code implement a timer such that you gradually grow a tract out of your 
seed point(the point you chose within the image), the code also implement a mouse callback such that 
each time you click somewhere on the screen, a new tract is created.

