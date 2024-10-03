Matrix4f Object::model() {
    Matrix4f scaleMatrix = Matrix4f::Identity();
    Matrix4f rotationMatrix = Matrix4f::Identity();
    Matrix4f translationMatrix = Matrix4f::Identity();

    // Create the translation matrix
    translationMatrix.block<3, 1>(0, 3) = center;

    // Create the scaling matrix
    scaleMatrix(0, 0) = scaling.x();
    scaleMatrix(1, 1) = scaling.y();
    scaleMatrix(2, 2) = scaling.z();
 
    // Create the rotation matrix
    const Quaternionf& r = rotation;
    rotationMatrix.block<3, 3>(0, 0) = r.toRotationMatrix();

# ifdef DEBUG
    // Output the matrix by using logger
    logger->info("scaling matrix: \n{}", scaleMatrix);
    logger->info("rotation matrix: \n{}", rotationMatrix);  
    logger->info("center matrix: \n{}", translationMatrix);
# endif

    Matrix4f modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
    return modelMatrix;
}