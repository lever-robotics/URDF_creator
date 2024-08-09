import React from 'react';
import './page1.css';
/**
 * Page1 component that displays the welcome message and description for the URDF Creator tool.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Page1 = () => {
    return (
        <div className="page1-container">
            <h1 className="welcome-header">
                <span className="welcome-text">Welcome</span> to URDF Creator
            </h1>
            <p className="description">
                An open-source tool for ROS2 developers. This tool assists in defining robots ready for high fidelity physics simulations.
            </p>
            <p className="description">
                This tool helps to take models from CAD modeling software such as those seen below and more and help define the software model the robot will utilize.
            </p>
            <p className="description">
                This tool is designed for ROS2 but the URDF model designed can be used on many platforms such as seen below.
            </p>
            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/roboeverything.png'} alt="Robo Everything" className="roboeverything-graphic" />
            </div>
        </div>
    );
};

export default Page1;