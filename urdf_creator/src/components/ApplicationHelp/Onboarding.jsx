import React, { useState } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faArrowLeft, faArrowRight } from '@fortawesome/free-solid-svg-icons';
import Page1 from './Page1';
import Page2 from './Page2';
import Page3 from './Page3';
import './onboardstyle.css';

const Onboarding = ({ closeOnboarding }) => {
    const [currentPage, setCurrentPage] = useState(1);
    const totalPages = 3;

    const handleNextPage = () => {
        if (currentPage < totalPages) {
            setCurrentPage(currentPage + 1);
        } else {
            closeOnboarding();
        }
    };

    const handlePreviousPage = () => {
        if (currentPage > 1) {
            setCurrentPage(currentPage - 1);
        }
    };

    const renderPage = () => {
        switch (currentPage) {
            case 1:
                return <Page1 />;
            case 2:
                return <Page2 />;
            case 3:
                return <Page3 />;
            default:
                return null;
        }
    };

    const renderDots = () => {
        return (
            <div className="dots-container">
                {[...Array(totalPages)].map((_, index) => (
                    <span
                        key={index}
                        className={`dot ${currentPage === index + 1 ? 'active' : ''}`}
                    ></span>
                ))}
            </div>
        );
    };

    return (
        <div className="onboarding">
            {renderPage()}
            <div className="bottom-section">
                <div className="arrow-container">
                    <button
                        className={`previous-button ${currentPage === 1 ? 'hidden' : ''}`}
                        onClick={handlePreviousPage}
                        disabled={currentPage === 1}
                    >
                        <FontAwesomeIcon icon={faArrowLeft} />
                    </button>
                    {renderDots()}
                    <button
                        className={`next-button`}
                        onClick={handleNextPage}
                    >
                        <FontAwesomeIcon icon={faArrowRight} />
                    </button>
                </div>
            </div>
        </div>
    );
};

export default Onboarding;