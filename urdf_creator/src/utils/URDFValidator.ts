// URDFValidator.ts

/**
 * Checks if a given URDF string is valid XML.
 * @param {string} urdfString - The URDF string to be checked.
 * @returns {boolean} True if the URDF string is valid XML, otherwise false.
 */
export function isValidURDF(urdfString: string): boolean {
    // Use DOMParser to parse the string as XML
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(urdfString, "application/xml");

    // Check for parser errors
    const parserErrorNS = xmlDoc.getElementsByTagName("parsererror").length;
    const parseError = xmlDoc.querySelector("parsererror");

    if (parserErrorNS > 0 || parseError !== null) {
        // If there's a parsererror element or the querySelector finds an error, the XML is invalid
        return false;
    }

    // Additionally, you can check if the document element is what you expect (e.g., <robot>)
    if (
        xmlDoc.documentElement.nodeName === "parsererror" ||
        xmlDoc.documentElement.nodeName !== "robot"
    ) {
        return false;
    }

    // If there are no errors and the document element is 'robot', the URDF is considered valid
    return true;
}

// string, boolean, number, Frame, StateFunctionsType
