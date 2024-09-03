import React from 'react';
import './onboarding.css';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faCog, faRocket, faCubes, faCode, faSpaceShuttle, faHome  } from '@fortawesome/free-solid-svg-icons';

/**
 * Page1 component that displays the welcome message and description for the URDF Creator tool.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Page1 = () => {
    return (
        <div className="page-container">
            <h1 className="welcome-header">
                <span className="blue">Welcome</span> to URDF Creator
            </h1>
            <div className="bullet-container">
                <ul className="bullet-list">
                    <li><FontAwesomeIcon icon={faCode} className="bullet-icon" />Free and Open Source</li>
                    <li><FontAwesomeIcon icon={faSpaceShuttle} className="bullet-icon" />Robot Developer Tool</li>
                    <li><FontAwesomeIcon icon={faHome} className="bullet-icon" />Designed for ROS2</li>
                    <li><FontAwesomeIcon icon={faCubes} className="bullet-icon" />Prepare CAD Robot Models</li>
                    <li><FontAwesomeIcon icon={faRocket} className="bullet-icon" />Ready for Robot Physics Sims</li>
                    <li><FontAwesomeIcon icon={faCog} className="bullet-icon" />Configure URDFs</li>
                </ul>
            </div>
            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/roboeverything3.png'} alt="Robo Everything" className="roboeverything-graphic" />
            </div>
        </div>
    );
};

export default Page1;
