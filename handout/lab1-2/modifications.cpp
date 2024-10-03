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
    Eigen::Matrix3f rotationMatrix3;
    const Quaternionf& r = rotation;
    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(),r.y(), r.z());
    rotationMatrix3 = (Eigen::AngleAxisf(radians(x_angle), Vector3f(1, 0, 0)) 
                        * Eigen::AngleAxisf(radians(y_angle), Vector3f(0, 1, 0)) 
                        * Eigen::AngleAxisf(radians(z_angle), Vector3f(0, 0, 1))).toRotationMatrix();
    rotationMatrix.block<3, 3>(0, 0) = rotationMatrix3;

# ifdef DEBUG
    // Output the matrix by using logger
    logger->info("scaling matrix: \n{}", scaleMatrix);
    logger->info("rotation matrix: \n{}", rotationMatrix);  
    logger->info("center matrix: \n{}", translationMatrix);
# endif

    Matrix4f modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
    return modelMatrix;
}