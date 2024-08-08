import React, { useState } from 'react';
import Page1 from './Page1';
import Page2 from './Page2';
import Page3 from './Page3';
import './onboardstyle.css';

/**
 * Onboarding component that handles the navigation through multiple pages.
 * 
 * @param {Object} props - The properties object.
 * @param {Function} props.closeOnboarding - Function to call when the onboarding process is complete.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Onboarding = ({ closeOnboarding }) => {
    const [currentPage, setCurrentPage] = useState(1);
    const totalPages = 3;

    /**
     * Handles the navigation to the next page.
     * If the current page is the last page, it calls the closeOnboarding function.
     */
    const handleNextPage = () => {
        if (currentPage < totalPages) {
            setCurrentPage(currentPage + 1);
        } else {
            closeOnboarding();
        }
    };

    /**
     * Handles the navigation to the previous page.
     */
    const handlePreviousPage = () => {
        if (currentPage > 1) {
            setCurrentPage(currentPage - 1);
        }
    };

    /**
     * Renders the current page based on the currentPage state.
     * 
     * @returns {JSX.Element|null} The component for the current page.
     */
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

    /**
     * Renders the dots indicating the current page.
     * 
     * @returns {JSX.Element} The dots container.
     */
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
            <div className="arrow-container">
                {currentPage > 1 && (
                    <button className="previous-button" onClick={handlePreviousPage}>
                        Previous
                    </button>
                )}
                {currentPage < totalPages && (
                    <button className="next-button" onClick={handleNextPage}>
                        Next
                    </button>
                )}
            </div>
            {renderDots()}
        </div>
    );
};

export default Onboarding;