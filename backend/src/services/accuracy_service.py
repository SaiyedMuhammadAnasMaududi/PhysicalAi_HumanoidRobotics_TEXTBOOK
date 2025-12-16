"""
Accuracy verification for responses against book content
"""
from typing import List, Dict, Any, Optional
from ..models.response import Response
from ..models.retrieval import RetrievalResult
from ..schemas.response import Citation


class AccuracyService:
    """Service for verifying the accuracy of responses against book content"""

    @staticmethod
    def verify_response_accuracy(
        response_text: str,
        citations: List[Dict[str, Any]],
        retrieved_content: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Verify that the response is accurate based on the retrieved content and citations
        """
        if not response_text or not citations:
            return {
                "is_accurate": False,
                "confidence_score": 0.0,
                "issues": ["Response is empty or has no citations"],
                "details": "Response must have content and proper citations to book content"
            }

        # Check if citations are properly formatted
        citation_issues = []
        for citation in citations:
            if not all(key in citation for key in ["chapter", "section", "similarity_score"]):
                citation_issues.append(f"Missing required fields in citation: {citation}")

        # Check if similarity scores are reasonable
        low_similarity_citations = [
            c for c in citations if c.get("similarity_score", 0) < 0.3
        ]
        if low_similarity_citations:
            citation_issues.append(
                f"Found {len(low_similarity_citations)} citations with low similarity scores (< 0.3)"
            )

        # Check if response content aligns with retrieved content
        alignment_score = AccuracyService._calculate_content_alignment(
            response_text,
            retrieved_content
        )

        # Calculate overall accuracy
        accuracy_issues = citation_issues.copy()
        is_accurate = True

        if alignment_score < 0.5:
            accuracy_issues.append(
                f"Low content alignment score: {alignment_score:.2f} "
                f"(should be ≥ 0.5 for acceptable alignment)"
            )
            is_accurate = False

        if len(citations) == 0:
            accuracy_issues.append("No citations provided")
            is_accurate = False

        return {
            "is_accurate": is_accurate and len(accuracy_issues) == 0,
            "confidence_score": alignment_score,
            "issues": accuracy_issues,
            "details": {
                "content_alignment_score": alignment_score,
                "citation_count": len(citations),
                "low_similarity_count": len(low_similarity_citations)
            }
        }

    @staticmethod
    def _calculate_content_alignment(
        response_text: str,
        retrieved_content: List[Dict[str, Any]]
    ) -> float:
        """
        Calculate how well the response aligns with the retrieved content
        This is a simplified implementation - a full implementation would use more sophisticated NLP
        """
        if not retrieved_content:
            return 0.0

        response_lower = response_text.lower()
        total_content = " ".join([item["content_text"].lower() for item in retrieved_content])

        # Count how many words from the response appear in the retrieved content
        response_words = set(response_lower.split())
        content_words = set(total_content.split())

        if not response_words:
            return 0.0

        matching_words = response_words.intersection(content_words)
        alignment_score = len(matching_words) / len(response_words)

        # Also check for phrase alignment
        phrase_matches = 0
        total_phrases = 0

        # Check for 2-3 word phrases
        for i in range(len(response_lower.split()) - 1):
            phrase = " ".join(response_lower.split()[i:i+2])
            if phrase in total_content:
                phrase_matches += 1
            total_phrases += 1

        for i in range(len(response_lower.split()) - 2):
            phrase = " ".join(response_lower.split()[i:i+3])
            if phrase in total_content:
                phrase_matches += 1
            total_phrases += 1

        phrase_score = phrase_matches / total_phrases if total_phrases > 0 else 0.0

        # Combine word-level and phrase-level alignment
        final_score = (alignment_score * 0.4) + (phrase_score * 0.6)
        return min(final_score, 1.0)  # Ensure score doesn't exceed 1.0

    @staticmethod
    def validate_citations_format(citations: List[Dict[str, Any]]) -> bool:
        """
        Validate that citations are in the proper format
        """
        required_fields = {"chapter", "section", "similarity_score"}

        for citation in citations:
            if not isinstance(citation, dict):
                return False

            if not required_fields.issubset(citation.keys()):
                return False

            # Validate similarity score range
            similarity_score = citation.get("similarity_score", -1)
            if not (0 <= similarity_score <= 1):
                return False

        return True

    @staticmethod
    def calculate_source_coverage(
        response_text: str,
        retrieved_content: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Calculate how much of the response is covered by the retrieved sources
        """
        if not retrieved_content:
            return {
                "coverage_percentage": 0.0,
                "covered_content": [],
                "uncovered_content": [response_text]
            }

        response_sentences = response_text.split('. ')
        covered_sentences = []
        uncovered_sentences = []

        for sentence in response_sentences:
            sentence_clean = sentence.strip().lower()
            is_covered = False

            for content_item in retrieved_content:
                content_text = content_item["content_text"].lower()
                if sentence_clean in content_text or \
                   AccuracyService._sentence_contains_similar_content(sentence_clean, content_text):
                    covered_sentences.append(sentence)
                    is_covered = True
                    break

            if not is_covered:
                uncovered_sentences.append(sentence)

        coverage_percentage = len(covered_sentences) / len(response_sentences) * 100 if response_sentences else 0

        return {
            "coverage_percentage": coverage_percentage,
            "covered_content": covered_sentences,
            "uncovered_content": uncovered_sentences
        }

    @staticmethod
    def _sentence_contains_similar_content(sentence: str, content: str) -> bool:
        """
        Check if a sentence contains content similar to the provided content
        """
        sentence_words = set(sentence.split())
        content_words = set(content.split())

        # Calculate Jaccard similarity
        intersection = sentence_words.intersection(content_words)
        union = sentence_words.union(content_words)

        if len(union) == 0:
            return False

        jaccard_similarity = len(intersection) / len(union)
        return jaccard_similarity > 0.3  # Threshold for similarity

    @staticmethod
    def generate_accuracy_report(
        response: Response,
        retrieved_results: List[RetrievalResult],
        original_query: str
    ) -> Dict[str, Any]:
        """
        Generate a comprehensive accuracy report for a response
        """
        # Convert RetrievalResult objects to dictionaries
        retrieved_content = [
            {
                "content_text": result.content_snippet,
                "chapter": result.source_document.split('/')[0] if '/' in result.source_document else "Unknown",
                "section": result.source_document.split('/')[1] if '/' in result.source_document else "Unknown",
                "similarity_score": result.similarity_score
            }
            for result in retrieved_results
        ]

        # Verify accuracy
        accuracy_check = AccuracyService.verify_response_accuracy(
            response.response_text,
            response.source_citations,
            retrieved_content
        )

        # Calculate source coverage
        coverage_report = AccuracyService.calculate_source_coverage(
            response.response_text,
            retrieved_content
        )

        return {
            "query": original_query,
            "response_id": str(response.response_id),
            "accuracy_check": accuracy_check,
            "coverage_report": coverage_report,
            "overall_accuracy_score": (
                accuracy_check["confidence_score"] * 0.7 +
                (coverage_report["coverage_percentage"] / 100) * 0.3
            ),
            "timestamp": response.generation_timestamp.isoformat() if response.generation_timestamp else None
        }

    @staticmethod
    def is_response_accurate_enough(
        accuracy_report: Dict[str, Any],
        minimum_accuracy_threshold: float = 0.7
    ) -> bool:
        """
        Determine if a response meets the minimum accuracy threshold
        """
        return accuracy_report["overall_accuracy_score"] >= minimum_accuracy_threshold