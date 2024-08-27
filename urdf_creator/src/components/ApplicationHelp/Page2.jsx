import React from 'react';
import './page2.css';

/**
 * Page2 component that explains URDF and its components like Links and Joints.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Page2 = () => {
    return (
        <div className="page2-container">
            <h2 className="section-header">What is a URDF?</h2>
            <p className="description">
                URDF (Uniform Robot Description Format) serves as a standardized XML-based format for describing robots in ROS. It encompasses various elements such as links, joints, sensors, and visuals, all crucial for accurately representing a robot's physical structure and kinematics.
            </p>
            
            <h2 className="section-header">Links and Joints</h2>
            <div className="content-section">
                <h3 className="subsection-header">Links</h3>
                <p className="description">
                    <strong>Links</strong> represent physical components of the robot, such as wheels, arms, or sensors.
                </p>
                <div className="graphic-container">
                    <img src={process.env.PUBLIC_URL + '/statics/link.png'} alt="Links" className="graphic"/>
                </div>
            </div>

            <div className="content-section">
                <h3 className="subsection-header">Joints</h3>
                <p className="description">
                    <strong>Joints</strong> define the connections between these links, enabling movement and articulation.
                </p>
                <div className="graphic-container">
                    <img src={process.env.PUBLIC_URL + '/statics/joints.png'} alt="Joints" className="graphic"/>
                </div>
            </div>
        </div>
    );
};

export default Page2;
